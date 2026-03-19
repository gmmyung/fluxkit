//! Frame types used by FOC transforms and controllers.

/// Three-phase frame.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Abc<T> {
    /// Phase A component.
    pub a: T,
    /// Phase B component.
    pub b: T,
    /// Phase C component.
    pub c: T,
}

impl<T> Abc<T> {
    /// Creates a new `abc` vector.
    #[inline]
    pub const fn new(a: T, b: T, c: T) -> Self {
        Self { a, b, c }
    }

    /// Maps each component independently.
    #[inline]
    pub fn map<U>(self, mut f: impl FnMut(T) -> U) -> Abc<U> {
        Abc {
            a: f(self.a),
            b: f(self.b),
            c: f(self.c),
        }
    }

    /// Zips two vectors with a component-wise mapping function.
    #[inline]
    pub fn zip_map<U, V>(self, rhs: Abc<U>, mut f: impl FnMut(T, U) -> V) -> Abc<V> {
        Abc {
            a: f(self.a, rhs.a),
            b: f(self.b, rhs.b),
            c: f(self.c, rhs.c),
        }
    }
}

impl<T: Default> Abc<T> {
    /// Returns a zero-like vector using `T::default()`.
    #[inline]
    pub fn zero() -> Self {
        Self {
            a: T::default(),
            b: T::default(),
            c: T::default(),
        }
    }
}

impl<T> Abc<T>
where
    T: Copy + core::ops::Mul<f32, Output = T>,
{
    /// Scales all components by the same scalar factor.
    #[inline]
    pub fn scale(self, k: f32) -> Self {
        Self {
            a: self.a * k,
            b: self.b * k,
            c: self.c * k,
        }
    }
}

/// Stationary orthogonal frame.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct AlphaBeta<T> {
    /// Alpha axis component.
    pub alpha: T,
    /// Beta axis component.
    pub beta: T,
}

impl<T> AlphaBeta<T> {
    /// Creates a new `alpha-beta` vector.
    #[inline]
    pub const fn new(alpha: T, beta: T) -> Self {
        Self { alpha, beta }
    }

    /// Maps each component independently.
    #[inline]
    pub fn map<U>(self, mut f: impl FnMut(T) -> U) -> AlphaBeta<U> {
        AlphaBeta {
            alpha: f(self.alpha),
            beta: f(self.beta),
        }
    }

    /// Zips two vectors with a component-wise mapping function.
    #[inline]
    pub fn zip_map<U, V>(self, rhs: AlphaBeta<U>, mut f: impl FnMut(T, U) -> V) -> AlphaBeta<V> {
        AlphaBeta {
            alpha: f(self.alpha, rhs.alpha),
            beta: f(self.beta, rhs.beta),
        }
    }
}

impl<T: Default> AlphaBeta<T> {
    /// Returns a zero-like vector using `T::default()`.
    #[inline]
    pub fn zero() -> Self {
        Self {
            alpha: T::default(),
            beta: T::default(),
        }
    }
}

impl<T> AlphaBeta<T>
where
    T: Copy + core::ops::Mul<f32, Output = T>,
{
    /// Scales all components by the same scalar factor.
    #[inline]
    pub fn scale(self, k: f32) -> Self {
        Self {
            alpha: self.alpha * k,
            beta: self.beta * k,
        }
    }
}

/// Rotating orthogonal frame aligned with electrical angle.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Dq<T> {
    /// Direct-axis component.
    pub d: T,
    /// Quadrature-axis component.
    pub q: T,
}

impl<T> Dq<T> {
    /// Creates a new `dq` vector.
    #[inline]
    pub const fn new(d: T, q: T) -> Self {
        Self { d, q }
    }

    /// Maps each component independently.
    #[inline]
    pub fn map<U>(self, mut f: impl FnMut(T) -> U) -> Dq<U> {
        Dq {
            d: f(self.d),
            q: f(self.q),
        }
    }

    /// Zips two vectors with a component-wise mapping function.
    #[inline]
    pub fn zip_map<U, V>(self, rhs: Dq<U>, mut f: impl FnMut(T, U) -> V) -> Dq<V> {
        Dq {
            d: f(self.d, rhs.d),
            q: f(self.q, rhs.q),
        }
    }
}

impl<T: Default> Dq<T> {
    /// Returns a zero-like vector using `T::default()`.
    #[inline]
    pub fn zero() -> Self {
        Self {
            d: T::default(),
            q: T::default(),
        }
    }
}

impl<T> Dq<T>
where
    T: Copy + core::ops::Mul<f32, Output = T>,
{
    /// Scales all components by the same scalar factor.
    #[inline]
    pub fn scale(self, k: f32) -> Self {
        Self {
            d: self.d * k,
            q: self.q * k,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{Abc, AlphaBeta, Dq};

    #[test]
    fn frame_helpers_preserve_structure() {
        let abc = Abc::new(1.0_f32, 2.0, 3.0);
        assert_eq!(abc.map(|x| x * 2.0), Abc::new(2.0, 4.0, 6.0));
        assert_eq!(
            abc.zip_map(Abc::new(4.0, 5.0, 6.0), |a, b| a + b),
            Abc::new(5.0, 7.0, 9.0)
        );
        assert_eq!(AlphaBeta::<f32>::zero(), AlphaBeta::new(0.0, 0.0));
        assert_eq!(Dq::new(2.0_f32, -3.0).scale(0.5), Dq::new(1.0, -1.5));
    }
}
