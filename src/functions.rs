use crate::interface::{Matrix2x9, WithRestriction};
use nalgebra::{DMatrix, Matrix3, RealField, Scalar};

pub fn generate_matrix_from_correspondences<T>(
    correspondences: Vec<&dyn WithRestriction<T>>,
) -> DMatrix<T>
where
    T: num_traits::identities::Zero + Scalar + Clone,
{
    let matrixes = correspondences
        .iter()
        .map(|c| c.generate_restriction())
        .collect::<Vec<Matrix2x9<T>>>();

    let rows = std::cmp::max(2 * matrixes.len(), 9);
    let mut matrix = DMatrix::zeros(rows, 9);

    for (i, m) in matrixes.iter().enumerate() {
        matrix.set_row(i * 2, &m.row(0));
        matrix.set_row(i * 2 + 1, &m.row(1));
    }

    return matrix;
}

pub struct HomographySolution<T: Scalar> {
    pub matrix: Matrix3<T>,
    pub value: T,
}

pub fn solve<T: RealField + Copy>(matrix: DMatrix<T>) -> HomographySolution<T> {
    let svd = matrix.svd(false, true);
    let v_t = svd.v_t.unwrap();
    let n = v_t.nrows() - 1;
    let last_column = v_t.row(n);
    let solution = Matrix3::from_vec(last_column.iter().cloned().collect::<Vec<T>>()).transpose();

    return HomographySolution {
        matrix: solution,
        value: svd.singular_values[n],
    };
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geo::{Line, Point};
    use crate::pairs::{LinePair, PointPair};

    #[test]
    fn test_generate_matrix_from_correspondences() {
        let p1 = Point::new(1., 2.);
        let p2 = Point::new(3., 4.);
        let l1 = Line::new(1., 2., 3.);
        let l2 = Line::new(4., 5., 6.);

        let pp = PointPair { p1, p2 };
        let lp = LinePair { l1, l2 };

        let matrix = generate_matrix_from_correspondences(vec![&pp, &lp]);
        assert_eq!(matrix.nrows(), 9);
        assert_eq!(matrix.ncols(), 9);
    }

    #[test]
    fn test_solve() {
        let matrix = DMatrix::from_vec(
            4,
            9,
            vec![
                1., 2., 3., 4., 5., 6., 7., 8., 9., 1., 2., 3., 4., 5., 6., 7., 8., 9., 1., 2., 3.,
                4., 5., 6., 7., 8., 9., 1., 2., 3., 4., 5., 6., 7., 8., 9.,
            ],
        );

        let _solution = solve(matrix);
    }
}
