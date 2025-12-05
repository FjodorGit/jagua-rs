use geo::{LineString, Polygon, line_string, unary_union};
use nfp::{NFPConvex, Point, point};

pub const FULL_TREE: [Point; 15] = [
    Point { x: 0.0, y: 0.8 },
    Point { x: -0.125, y: 0.5 },
    Point { x: -0.0625, y: 0.5 },
    Point { x: -0.2, y: 0.25 },
    Point { x: -0.1, y: 0.25 },
    Point { x: -0.35, y: 0.0 },
    Point { x: -0.075, y: 0.0 },
    Point { x: -0.075, y: -0.2 },
    Point { x: 0.075, y: -0.2 },
    Point { x: 0.075, y: 0.0 },
    Point { x: 0.35, y: 0.0 },
    Point { x: 0.1, y: 0.25 },
    Point { x: 0.2, y: 0.25 },
    Point { x: 0.0625, y: 0.5 },
    Point { x: 0.125, y: 0.5 },
];

const SCALING_FACTOR: f64 = 10000.0;
const TOP_TIER: [Point; 3] = [
    Point { x: 0.0, y: 0.8 },
    Point { x: -0.125, y: 0.5 },
    Point { x: 0.125, y: 0.5 },
];

const MID_TIER: [Point; 4] = [
    Point { x: -0.0625, y: 0.5 },
    Point { x: -0.2, y: 0.25 },
    Point { x: 0.2, y: 0.25 },
    Point { x: 0.0625, y: 0.5 },
];

const LOW_TIER: [Point; 4] = [
    Point { x: -0.1, y: 0.25 },
    Point { x: -0.35, y: 0.0 },
    Point { x: 0.35, y: 0.0 },
    Point { x: 0.1, y: 0.25 },
];

const TRUNK: [Point; 4] = [
    Point { x: -0.075, y: 0.0 },
    Point { x: -0.075, y: -0.2 },
    Point { x: 0.075, y: -0.2 },
    Point { x: 0.075, y: 0.0 },
];

pub fn scale(poly: &[Point]) -> Vec<Point> {
    poly.iter()
        .map(|p| point(p.x * SCALING_FACTOR, p.y * SCALING_FACTOR))
        .collect()
}

pub fn rotate(points: &[Point], deg: f64) -> Vec<Point> {
    points
        .iter()
        .map(|p| {
            let new_x = deg.to_radians().cos() * p.x - deg.to_radians().sin() * p.y;
            let new_y = deg.to_radians().sin() * p.x + deg.to_radians().cos() * p.y;
            Point::new(new_x, new_y)
        })
        .collect()
}

pub fn christmas_tree(deg: f64) -> Vec<Point> {
    rotate(&scale(&FULL_TREE), deg)
}

pub fn christmas_tree_nfps(deg: f64) -> Vec<Vec<Point>> {
    let rot_scaled_top = rotate(&scale(&TOP_TIER), deg);
    let rot_scaled_middle = rotate(&scale(&MID_TIER), deg);
    let rot_scaled_low = rotate(&scale(&LOW_TIER), deg);
    let rot_scaled_trunk = rotate(&scale(&TRUNK), deg);

    let scaled_top = scale(&TOP_TIER);
    let scaled_middle = scale(&MID_TIER);
    let scaled_low = scale(&LOW_TIER);
    let scaled_trunk = scale(&TRUNK);

    let convex_decomp = [scaled_top, scaled_middle, scaled_low, scaled_trunk];
    let rot_convex_decomp = [
        rot_scaled_top,
        rot_scaled_middle,
        rot_scaled_low,
        rot_scaled_trunk,
    ];

    let names = ["top", "middle", "low", "trunk"];

    std::fs::create_dir_all("output").ok();

    let mut all_nfps = Vec::new();
    let mut pair_idx = 0;
    let mut polygons = vec![];
    for (i, poly_a) in rot_convex_decomp.iter().enumerate() {
        for (j, poly_b) in convex_decomp.iter().enumerate() {
            let mut nfp = NFPConvex::nfp(poly_b, poly_a).unwrap();
            let point_tuples = nfp.iter().map(|p| (p.x, p.y)).collect::<Vec<(f64, f64)>>();
            let linestring: LineString<f64> = point_tuples.into();
            let polygon = Polygon::new(linestring, vec![]);
            polygons.push(polygon);
            let filename = format!("output/nfp_{}_{}_to_{}.svg", pair_idx, names[j], names[i]);
            write_nfp_svg(&nfp, &filename, poly_a, poly_b);
            nfp.push(nfp[0].clone());
            all_nfps.push(nfp);
            pair_idx += 1;
        }
    }

    let polygon_union = unary_union(&polygons)
        .0
        .pop()
        .expect("should have a polygon");

    write_all_nfps_svg(&polygon_union, &convex_decomp, "output/all_nfps.svg");

    all_nfps
}

fn write_nfp_svg(nfp: &[Point], filename: &str, poly_a: &[Point], poly_b: &[Point]) {
    let all_points: Vec<&Point> = nfp
        .iter()
        .chain(poly_a.iter())
        .chain(poly_b.iter())
        .collect();

    let min_x = all_points.iter().map(|p| p.x).fold(f64::INFINITY, f64::min);
    let max_x = all_points
        .iter()
        .map(|p| p.x)
        .fold(f64::NEG_INFINITY, f64::max);
    let min_y = all_points.iter().map(|p| p.y).fold(f64::INFINITY, f64::min);
    let max_y = all_points
        .iter()
        .map(|p| p.y)
        .fold(f64::NEG_INFINITY, f64::max);

    let margin = 500.0;
    let width = max_x - min_x + 2.0 * margin;
    let height = max_y - min_y + 2.0 * margin;

    let mut svg = format!(
        r#"<svg xmlns="http://www.w3.org/2000/svg" viewBox="{} {} {} {}" width="800" height="800">
<rect width="100%" height="100%" fill="white"/>
"#,
        min_x - margin,
        min_y - margin,
        width,
        height
    );

    svg.push_str(&format_polygon(poly_a, "blue", 0.3));
    svg.push_str(&format_polygon(poly_b, "green", 0.3));
    svg.push_str(&format_polygon(nfp, "red", 0.7));

    svg.push_str("</svg>");

    std::fs::write(filename, svg).unwrap();
}

fn format_polygon(points: &[Point], color: &str, opacity: f64) -> String {
    if points.is_empty() {
        return String::new();
    }

    let points_str: String = points
        .iter()
        .map(|p| format!("{},{}", p.x, p.y))
        .collect::<Vec<_>>()
        .join(" ");

    format!(
        r#"<polygon points="{}" fill="{}" stroke="black" stroke-width="20" fill-opacity="{}" />
"#,
        points_str, color, opacity
    )
}

fn write_all_nfps_svg(union_polygon: &Polygon<f64>, convex_shapes: &[Vec<Point>], filename: &str) {
    let union_coords: Vec<_> = union_polygon.exterior().coords().collect();

    let all_x: Vec<f64> = union_coords
        .iter()
        .map(|c| c.x)
        .chain(
            convex_shapes
                .iter()
                .flat_map(|shape| shape.iter().map(|p| p.x)),
        )
        .collect();
    let all_y: Vec<f64> = union_coords
        .iter()
        .map(|c| c.y)
        .chain(
            convex_shapes
                .iter()
                .flat_map(|shape| shape.iter().map(|p| p.y)),
        )
        .collect();

    let min_x = all_x.iter().copied().fold(f64::INFINITY, f64::min);
    let max_x = all_x.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let min_y = all_y.iter().copied().fold(f64::INFINITY, f64::min);
    let max_y = all_y.iter().copied().fold(f64::NEG_INFINITY, f64::max);

    let margin = 1000.0;
    let width = max_x - min_x + 2.0 * margin;
    let height = max_y - min_y + 2.0 * margin;

    let mut svg = format!(
        r#"<svg xmlns="http://www.w3.org/2000/svg" viewBox="{} {} {} {}">
"#,
        min_x - margin,
        min_y - margin,
        width,
        height
    );

    for shape in convex_shapes.iter() {
        svg.push_str(&format_polygon(shape, "blue", 0.5));
    }

    svg.push_str(&format_geo_polygon(union_polygon, "red", 0.5));

    svg.push_str("</svg>");

    std::fs::write(filename, svg).unwrap();
}

fn format_geo_polygon(polygon: &Polygon<f64>, color: &str, opacity: f64) -> String {
    let points_str: String = polygon
        .exterior()
        .coords()
        .map(|c| format!("{},{}", c.x, c.y))
        .collect::<Vec<_>>()
        .join(" ");

    format!(
        r#"<polygon points="{}" fill="{}" stroke="black" stroke-width="20" fill-opacity="{}" />
"#,
        points_str, color, opacity
    )
}
