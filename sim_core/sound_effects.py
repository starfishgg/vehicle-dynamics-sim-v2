"""
sound_effect.py

Procedural engine sound synthesis.

This module creates a synthetic four-cylinder, four-stroke petrol engine
sound from engine RPM, throttle and torque.

The sound is built from several components:

    - Four individual cylinder combustion events
    - Combustion harmonics
    - Crankshaft / mechanical noise
    - Intake sound
    - Exhaust sound
    - Small natural variation between combustion events

The goal is not to reproduce a particular real engine recording.

Instead, the system models some of the important physical relationships
that make an engine sound like an engine:

    RPM -> firing frequency -> pitch

    Throttle/load -> combustion intensity

    RPM/load -> harmonic content

    Throttle -> intake and exhaust character
"""

import math
import threading

import numpy as np
import sounddevice as sd


class EngineSoundEffect:
    """Generate a procedural four-cylinder engine sound."""

    SAMPLE_RATE = 44100
    CHANNELS = 1
    BLOCK_SIZE = 512

    MIN_RPM = 700.0
    MAX_RPM = 7000.0

    CYLINDER_COUNT = 4

    # Typical firing order for a four-cylinder engine.
    #
    # The exact order is not universal, but 1-3-4-2 is common.
    FIRING_ORDER = (0, 2, 3, 1)

    def __init__(
        self,
        max_torque: float = 400.0,
        master_volume: float = 0.25,
    ) -> None:
        self.max_torque = max_torque
        self.master_volume = master_volume

        # --------------------------------------------------------------
        # Simulator inputs
        # --------------------------------------------------------------

        self.target_rpm = self.MIN_RPM
        self.target_throttle = 0.0
        self.target_torque = 0.0
        self.target_clutch = 1.0

        # --------------------------------------------------------------
        # Smoothed audio values
        # --------------------------------------------------------------

        self.audio_rpm = self.MIN_RPM
        self.audio_throttle = 0.0
        self.audio_torque = 0.0
        self.audio_clutch = 1.0

        # --------------------------------------------------------------
        # Engine phases
        # --------------------------------------------------------------

        # One complete four-stroke engine cycle is 720 degrees of
        # crankshaft rotation.
        #
        # We represent the engine cycle as 0.0 -> 1.0.
        self.engine_cycle_phase = 0.0

        # Separate phase for mechanical rotation.
        self.crank_phase = 0.0

        # --------------------------------------------------------------
        # Cylinder characteristics
        # --------------------------------------------------------------

        # Each cylinder gets a slightly different character.
        #
        # Real cylinders do not produce perfectly identical pressure
        # pulses. Small differences make the sound less synthetic.
        self.cylinder_amplitudes = np.array(
            [1.00, 0.96, 1.04, 0.98],
            dtype=np.float64,
        )

        self.cylinder_decay = np.array(
            [1.00, 0.94, 1.06, 0.98],
            dtype=np.float64,
        )

        # --------------------------------------------------------------
        # Noise state
        # --------------------------------------------------------------

        self.intake_noise_state = 0.0
        self.exhaust_noise_state = 0.0
        self.mechanical_noise_state = 0.0

        # --------------------------------------------------------------
        # Audio stream
        # --------------------------------------------------------------

        self._stream = None
        self._lock = threading.Lock()

    # ==================================================================
    # PUBLIC INTERFACE
    # ==================================================================

    def start(self) -> None:
        """Start the audio stream."""

        if self._stream is not None:
            return

        self._stream = sd.OutputStream(
            samplerate=self.SAMPLE_RATE,
            channels=self.CHANNELS,
            dtype="float32",
            blocksize=self.BLOCK_SIZE,
            callback=self._audio_callback,
        )

        self._stream.start()

    def stop(self) -> None:
        """Stop and close the audio stream."""

        if self._stream is None:
            return

        self._stream.stop()
        self._stream.close()
        self._stream = None

    def update(
        self,
        rpm: float,
        throttle: float,
        engine_torque: float,
        clutch: float = 1.0,
    ) -> None:
        """
        Update the sound generator.

        The vehicle simulation should call this regularly.

        RPM controls pitch.

        Throttle and torque affect combustion, intake and exhaust.

        Clutch affects how strongly engine load is represented.
        """

        with self._lock:
            self.target_rpm = np.clip(
                rpm,
                self.MIN_RPM,
                self.MAX_RPM,
            )

            self.target_throttle = np.clip(
                throttle,
                0.0,
                1.0,
            )

            self.target_torque = engine_torque

            self.target_clutch = np.clip(
                clutch,
                0.0,
                1.0,
            )

    # ==================================================================
    # AUDIO CALLBACK
    # ==================================================================

    def _audio_callback(
        self,
        outdata: np.ndarray,
        frames: int,
        time_info,
        status,
    ) -> None:
        """Generate one block of audio."""

        if status:
            print(status)

        with self._lock:
            target_rpm = self.target_rpm
            target_throttle = self.target_throttle
            target_torque = self.target_torque
            target_clutch = self.target_clutch

        # Smooth simulator values across the entire audio block.
        rpm_values = np.linspace(
            self.audio_rpm,
            target_rpm,
            frames,
            endpoint=False,
        )

        throttle_values = np.linspace(
            self.audio_throttle,
            target_throttle,
            frames,
            endpoint=False,
        )

        torque_values = np.linspace(
            self.audio_torque,
            target_torque,
            frames,
            endpoint=False,
        )

        clutch_values = np.linspace(
            self.audio_clutch,
            target_clutch,
            frames,
            endpoint=False,
        )

        self.audio_rpm = target_rpm
        self.audio_throttle = target_throttle
        self.audio_torque = target_torque
        self.audio_clutch = target_clutch

        # Generate the engine layers.
        combustion = self._generate_combustion(
            rpm_values,
            throttle_values,
            torque_values,
        )

        mechanical = self._generate_mechanical(
            rpm_values,
            throttle_values,
        )

        intake = self._generate_intake(
            rpm_values,
            throttle_values,
        )

        exhaust = self._generate_exhaust(
            rpm_values,
            throttle_values,
            torque_values,
        )

        # --------------------------------------------------------------
        # Combine the engine
        # --------------------------------------------------------------

        engine_sound = (
            combustion
            + mechanical
            + intake
            + exhaust
        )

        # Clutch disengagement reduces the mechanical load slightly.
        engine_sound *= (
            0.85
            + (0.15 * clutch_values)
        )

        # Soft saturation prevents loud combinations of layers from
        # producing ugly digital clipping.
        engine_sound = np.tanh(
            engine_sound * 1.8
        )

        engine_sound *= self.master_volume

        outdata[:, 0] = engine_sound.astype(
            np.float32
        )

    # ==================================================================
    # COMBUSTION
    # ==================================================================

    def _generate_combustion(
        self,
        rpm_values: np.ndarray,
        throttle_values: np.ndarray,
        torque_values: np.ndarray,
    ) -> np.ndarray:
        """
        Generate individual combustion events.

        A four-stroke engine completes one cycle every two crankshaft
        revolutions.

        Four cylinders therefore produce:

            4 / 2 = 2

        combustion events per crankshaft revolution.

        At 3000 RPM:

            3000 / 60 * 2 = 100 combustion events/sec
        """

        frames = len(rpm_values)

        combustion = np.zeros(
            frames,
            dtype=np.float64,
        )

        phase = self.engine_cycle_phase

        for index in range(frames):
            rpm = rpm_values[index]
            throttle = throttle_values[index]
            torque = torque_values[index]

            # ----------------------------------------------------------
            # Advance engine cycle
            # ----------------------------------------------------------

            cycle_frequency = rpm / 120.0

            phase += cycle_frequency / self.SAMPLE_RATE
            phase %= 1.0

            # ----------------------------------------------------------
            # Determine whether each cylinder is currently firing.
            # ----------------------------------------------------------

            sample = 0.0

            for cylinder_index in range(
                self.CYLINDER_COUNT
            ):
                firing_position = self._cylinder_firing_position(
                    cylinder_index
                )

                # Distance around the circular phase.
                distance = (
                    phase
                    - firing_position
                ) % 1.0

                # A combustion event is strongest immediately after
                # the firing point and then decays.
                pulse = math.exp(
                    -distance
                    * 80.0
                    * self.cylinder_decay[cylinder_index]
                )

                # A small second component gives the combustion event
                # some sharper high-frequency character.
                sharp_pulse = math.exp(
                    -distance
                    * 300.0
                )

                # ------------------------------------------------------
                # Engine load
                # ------------------------------------------------------

                torque_load = min(
                    abs(torque) / self.max_torque,
                    1.0,
                )

                load = max(
                    throttle,
                    torque_load,
                )

                # At idle the combustion pulse is still present, but
                # substantially weaker.
                combustion_strength = (
                    0.18
                    + load * 0.82
                )

                # High RPM produces slightly harder, sharper pulses.
                rpm_factor = (
                    0.85
                    + 0.15
                    * (
                        rpm / self.MAX_RPM
                    )
                )

                cylinder_strength = (
                    self.cylinder_amplitudes[
                        cylinder_index
                    ]
                    * combustion_strength
                    * rpm_factor
                )

                sample += (
                    pulse * cylinder_strength
                    + sharp_pulse
                    * cylinder_strength
                    * 0.18
                )

            combustion[index] = sample

        self.engine_cycle_phase = phase

        # --------------------------------------------------------------
        # Add harmonic content.
        #
        # This is based on crankshaft/firing frequency rather than
        # simply adding arbitrary fixed frequencies.
        # --------------------------------------------------------------

        firing_frequency = rpm_values / 30.0

        # Use the accumulated engine-cycle phase for the harmonic layer.
        harmonic_phase = (
            self.engine_cycle_phase
        )

        # The main pulse already contains a lot of harmonic content.
        # These additional components give the engine a harder edge.
        harmonic_1 = np.sin(
            2.0
            * math.pi
            * harmonic_phase
            * 4.0
        )

        harmonic_2 = np.sin(
            2.0
            * math.pi
            * harmonic_phase
            * 8.0
        )

        harmonic_3 = np.sin(
            2.0
            * math.pi
            * harmonic_phase
            * 12.0
        )

        # Higher RPM produces more high-frequency energy.
        high_rpm = np.clip(
            (rpm_values - 1500.0)
            / 5500.0,
            0.0,
            1.0,
        )

        harmonic_mix = (
            harmonic_1 * 0.035
            + harmonic_2
            * (
                0.018
                + high_rpm * 0.025
            )
            + harmonic_3
            * (
                0.008
                + high_rpm * 0.015
            )
        )

        combustion += harmonic_mix

        return combustion

    # ==================================================================
    # CYLINDER FIRING
    # ==================================================================

    def _cylinder_firing_position(
        self,
        cylinder_index: int,
    ) -> float:
        """
        Return the firing position of a cylinder within the 720-degree
        engine cycle.

        Four cylinders fire evenly throughout the 720-degree cycle.

        The firing order determines which cylinder occupies each firing
        position.
        """

        for firing_index, cylinder in enumerate(
            self.FIRING_ORDER
        ):
            if cylinder == cylinder_index:
                return firing_index / self.CYLINDER_COUNT

        return 0.0

    # ==================================================================
    # MECHANICAL SOUND
    # ==================================================================

    def _generate_mechanical(
        self,
        rpm_values: np.ndarray,
        throttle_values: np.ndarray,
    ) -> np.ndarray:
        """Generate crankshaft and mechanical engine sounds."""

        frames = len(rpm_values)

        mechanical = np.zeros(
            frames,
            dtype=np.float64,
        )

        phase = self.crank_phase

        for index in range(frames):
            rpm = rpm_values[index]

            crank_frequency = rpm / 60.0

            phase += (
                crank_frequency
                / self.SAMPLE_RATE
            )

            phase %= 1.0

            fundamental = math.sin(
                2.0
                * math.pi
                * phase
            )

            second = math.sin(
                2.0
                * math.pi
                * phase
                * 2.0
            )

            third = math.sin(
                2.0
                * math.pi
                * phase
                * 3.0
            )

            mechanical[index] = (
                fundamental * 0.055
                + second * 0.028
                + third * 0.012
            )

        self.crank_phase = phase

        # Mechanical sound becomes relatively more noticeable during
        # low-throttle operation.
        mechanical *= (
            1.0
            - throttle_values * 0.25
        )

        return mechanical

    # ==================================================================
    # INTAKE
    # ==================================================================

    def _generate_intake(
        self,
        rpm_values: np.ndarray,
        throttle_values: np.ndarray,
    ) -> np.ndarray:
        """
        Generate intake airflow noise.

        Throttle is the primary control here. Opening the throttle should
        make the engine sound like it is drawing in considerably more air.
        """

        frames = len(rpm_values)

        random_noise = np.random.uniform(
            -1.0,
            1.0,
            frames,
        )

        intake = np.empty(
            frames,
            dtype=np.float64,
        )

        noise_state = self.intake_noise_state

        for index in range(frames):
            noise_state += (
                random_noise[index]
                - noise_state
            ) * 0.12

            intake[index] = noise_state

        self.intake_noise_state = noise_state

        # Intake gets substantially louder with throttle.
        throttle_strength = (
            throttle_values ** 1.5
        )

        # Higher RPM also increases airflow noise.
        rpm_strength = np.clip(
            rpm_values / self.MAX_RPM,
            0.0,
            1.0,
        )

        intake *= (
            throttle_strength
            * (
                0.04
                + rpm_strength * 0.08
            )
        )

        return intake

    # ==================================================================
    # EXHAUST
    # ==================================================================

    def _generate_exhaust(
        self,
        rpm_values: np.ndarray,
        throttle_values: np.ndarray,
        torque_values: np.ndarray,
    ) -> np.ndarray:
        """Generate exhaust/combustion noise."""

        frames = len(rpm_values)

        random_noise = np.random.uniform(
            -1.0,
            1.0,
            frames,
        )

        exhaust = np.empty(
            frames,
            dtype=np.float64,
        )

        noise_state = self.exhaust_noise_state

        for index in range(frames):
            noise_state += (
                random_noise[index]
                - noise_state
            ) * 0.045

            exhaust[index] = noise_state

        self.exhaust_noise_state = noise_state

        torque_load = np.clip(
            np.abs(torque_values)
            / self.max_torque,
            0.0,
            1.0,
        )

        load = np.maximum(
            throttle_values,
            torque_load,
        )

        # Exhaust becomes stronger with engine load.
        exhaust *= (
            0.025
            + load * 0.10
        )

        # Add a low-frequency exhaust pulse related to engine speed.
        exhaust_frequency = rpm_values / 30.0

        exhaust_phase = (
            self.engine_cycle_phase
        )

        exhaust_pulse = np.sin(
            2.0
            * math.pi
            * exhaust_phase
            * 4.0
        )

        exhaust += (
            exhaust_pulse
            * load
            * 0.035
        )

        return exhaust


if __name__ == "__main__":
    # Simple standalone test.
    #
    # This lets us test the sound generator without involving the vehicle
    # simulator yet.

    import time

    engine_sound = EngineSoundEffect()

    engine_sound.start()

    try:
        # --------------------------------------------------------------
        # Idle
        # --------------------------------------------------------------

        engine_sound.update(
            rpm=850,
            throttle=0.0,
            engine_torque=140,
            clutch=1.0,
        )

        time.sleep(2.0)

        # --------------------------------------------------------------
        # Rev up
        # --------------------------------------------------------------

        for rpm in range(
            850,
            7001,
            20,
        ):
            throttle = min(
                1.0,
                (rpm - 850) / 2000.0,
            )

            engine_sound.update(
                rpm=rpm,
                throttle=throttle,
                engine_torque=400.0 * throttle,
                clutch=1.0,
            )

            time.sleep(0.01)

        # --------------------------------------------------------------
        # Hold redline
        # --------------------------------------------------------------

        time.sleep(1.5)

        # --------------------------------------------------------------
        # Rev down
        # --------------------------------------------------------------

        for rpm in range(
            7000,
            849,
            -20,
        ):
            throttle = max(
                0.0,
                (rpm - 850) / 2000.0,
            )

            engine_sound.update(
                rpm=rpm,
                throttle=throttle,
                engine_torque=400.0 * throttle,
                clutch=1.0,
            )

            time.sleep(0.01)

        # --------------------------------------------------------------
        # Return to idle
        # --------------------------------------------------------------

        engine_sound.update(
            rpm=850,
            throttle=0.0,
            engine_torque=140,
            clutch=1.0,
        )

        time.sleep(2.0)

    finally:
        engine_sound.stop()
