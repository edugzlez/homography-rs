use homography::geo::{Line, Point};
use homography::HomographyComputation;

fn main() {
    let w = 80.;
    let h = 60.;

    let p1 = Point::new(148., 337.);
    let p2 = Point::new(131., 516.);
    let p3 = Point::new(321., 486.);
    let p4 = Point::new(332., 370.);

    let p1p = Point::new(0., 0.);
    let p2p = Point::new(0., h);
    let p3p = Point::new(w, h);
    let p4p = Point::new(w, 0.);

    let line1 = Line::from_points(&p1, &p2);
    let line2 = Line::from_points(&p3, &p4);
    let line3 = Line::from_points(&p1, &p4);
    let line4 = Line::from_points(&p2, &p3);

    let line1p = Line::from_points(&p1p, &p2p);
    let line2p = Line::from_points(&p2p, &p3p);
    let line3p = Line::from_points(&p3p, &p4p);
    let line4p = Line::from_points(&p4p, &p1p);

    let mut hc = HomographyComputation::new();

    hc.add_point_correspondence(p1, p1p);
    hc.add_point_correspondence(p2, p2p);
    hc.add_point_correspondence(p3, p3p);
    hc.add_point_correspondence(p4, p4p);

    hc.add_line_correspondence(line1, line1p);
    hc.add_line_correspondence(line2, line2p);
    hc.add_line_correspondence(line3, line3p);
    hc.add_line_correspondence(line4, line4p);

    let rs = hc.get_restrictions();
    let sol = rs.compute();

    println!("Matrix: {}", sol.matrix);
    println!("Value: {}", sol.value);
}
