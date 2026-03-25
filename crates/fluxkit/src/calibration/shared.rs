use fluxkit_core::{
    ActuatorBlendBandCalibrator, ActuatorBreakawayCalibrator, ActuatorCalibration,
    ActuatorFrictionCalibrator, ActuatorGearRatioCalibrator, CalibrationError,
    FluxLinkageCalibrator, MotorCalibration, PhaseInductanceCalibrator, PhaseResistanceCalibrator,
    PolePairsAndOffsetCalibrator,
};

pub(crate) trait RoutineState<R> {
    fn result(&self) -> Option<R>;
    fn error(&self) -> Option<CalibrationError>;
}

macro_rules! impl_routine_state {
    ($result:ty => $($ty:ty),+ $(,)?) => {
        $(
            impl RoutineState<$result> for $ty {
                #[inline]
                fn result(&self) -> Option<$result> {
                    self.result().map(Into::into)
                }

                #[inline]
                fn error(&self) -> Option<CalibrationError> {
                    self.error()
                }
            }
        )+
    };
}

impl_routine_state!(ActuatorCalibration => ActuatorGearRatioCalibrator,);
impl_routine_state!(ActuatorCalibration => ActuatorFrictionCalibrator,);
impl_routine_state!(ActuatorCalibration => ActuatorBreakawayCalibrator,);
impl_routine_state!(ActuatorCalibration => ActuatorBlendBandCalibrator,);
impl_routine_state!(MotorCalibration => PolePairsAndOffsetCalibrator,);
impl_routine_state!(MotorCalibration => PhaseResistanceCalibrator,);
impl_routine_state!(MotorCalibration => PhaseInductanceCalibrator,);
impl_routine_state!(MotorCalibration => FluxLinkageCalibrator,);
