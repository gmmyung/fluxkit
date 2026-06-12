use core::cell::RefCell;

use critical_section::Mutex;
use fluxkit_core::{
    ActuatorBlendBandCalibrator, ActuatorBreakawayCalibrator, ActuatorCalibration,
    ActuatorFrictionCalibrator, ActuatorGearRatioCalibrator, CalibrationError,
    FluxLinkageCalibrator, MotorCalibration, PhaseInductanceCalibrator, PhaseResistanceCalibrator,
    PolePairsAndOffsetCalibrator,
};

use crate::capability::take_active_inner;

#[derive(Clone, Copy, Debug)]
pub(crate) struct SharedStatus<S> {
    pub status: S,
}

#[inline]
pub(crate) fn read_status<S: Copy>(shared: &Mutex<RefCell<SharedStatus<S>>>) -> S {
    critical_section::with(|cs| shared.borrow(cs).borrow().status)
}

#[inline]
pub(crate) fn write_status<S: Copy>(
    shared: &Mutex<RefCell<SharedStatus<S>>>,
    update: impl FnOnce(&mut S),
) {
    critical_section::with(|cs| {
        let mut shared = shared.borrow(cs).borrow_mut();
        update(&mut shared.status);
    });
}

pub(crate) fn run_active_calibration_inner<T, S, E>(
    inner: &Mutex<RefCell<Option<T>>>,
    shared: &Mutex<RefCell<SharedStatus<S>>>,
    is_active: impl FnOnce() -> bool,
    error_from_active: impl FnOnce(bool) -> E,
    run: impl FnOnce(&mut T, &Mutex<RefCell<SharedStatus<S>>>) -> Result<(), E>,
) -> Result<(), E>
where
    S: Copy,
{
    let mut active_inner = take_active_inner(inner, is_active, error_from_active)?;
    let result = run(&mut active_inner, shared);
    critical_section::with(|cs| {
        *inner.borrow(cs).borrow_mut() = Some(active_inner);
    });
    result
}

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
