use crate::{
    geo::{Line, Point},
    interface::{Matrix2x9, WithRestriction},
};

use nalgebra::{ArrayStorage, Matrix, Scalar, U1, U9};
use num_traits::Float;

pub struct PointPair<T = f32> {
    pub p1: Point<T>,
    pub p2: Point<T>,
}

pub struct LinePair<T = f32> {
    pub l1: Line<T>,
    pub l2: Line<T>,
}

type RowVector9<T> = Matrix<T, U1, U9, ArrayStorage<T, 1, 9>>;

impl<T: Float + Scalar> WithRestriction<T> for PointPair<T> {
    fn generate_restriction(&self) -> Matrix2x9<T> {
        let p1 = &self.p1;
        let p2 = &self.p2;

        let first_row = RowVector9::from_vec(vec![
            T::zero(),
            T::zero(),
            T::zero(),
            -p1.x,
            -p1.y,
            -T::one(),
            p1.x * p2.y,
            p1.y * p2.y,
            p2.y,
        ]);

        let second_row = RowVector9::from_vec(vec![
            p1.x,
            p1.y,
            T::one(),
            T::zero(),
            T::zero(),
            T::zero(),
            -p1.x * p2.x,
            -p1.y * p2.x,
            -p2.x,
        ]);

        Matrix2x9::from_rows(&vec![first_row, second_row])
    }
}

impl<T: Float + Scalar> WithRestriction<T> for LinePair<T> {
    fn generate_restriction(&self) -> Matrix2x9<T> {
        let l1 = &self.l1;
        let l2 = &self.l2;

        let first_row = RowVector9::from_vec(vec![
            T::zero(),
            -l1.c * l2.a,
            l1.b * l2.a,
            T::zero(),
            -l1.c * l2.b,
            l1.b * l2.b,
            T::zero(),
            -l1.c * l2.c,
            l1.b * l2.c,
        ]);

        let second_row = RowVector9::from_vec(vec![
            l1.c * l2.a,
            T::zero(),
            -l1.a * l2.a,
            l1.c * l2.b,
            T::zero(),
            -l1.a * l2.b,
            l1.c * l2.c,
            T::zero(),
            -l1.a * l2.c,
        ]);

        return Matrix2x9::from_rows(&vec![first_row, second_row]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_generate_restriction_point_pair() {
        let p1 = Point::new(1., 2.);
        let p2 = Point::new(3., 4.);
        let pair = PointPair { p1, p2 };

        let _restriction = pair.generate_restriction();
    }

    #[test]
    fn test_generate_restriction_line_pair() {
        let l1 = Line::new(1., 2., 3.);
        let l2 = Line::new(4., 5., 6.);
        let pair = LinePair { l1, l2 };

        let _restriction = pair.generate_restriction();
    }
}
