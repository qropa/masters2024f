#![allow(unused_imports)]
use itertools::{iproduct, Itertools};
use proconio::{marker::*, *};
use std::collections::*;
use std::{cmp::*, collections::*};
use svg::node::{
    element::{Circle, Group, Line, Rectangle, Title},
    Text,
};
use wasm_bindgen::prelude::*;

use std::ops::RangeBounds;

pub trait SetMinMax {
    fn setmin(&mut self, v: Self) -> bool;
    fn setmax(&mut self, v: Self) -> bool;
}
impl<T> SetMinMax for T
where
    T: PartialOrd,
{
    fn setmin(&mut self, v: T) -> bool {
        *self > v && {
            *self = v;
            true
        }
    }
    fn setmax(&mut self, v: T) -> bool {
        *self < v && {
            *self = v;
            true
        }
    }
}

#[macro_export]
macro_rules! mat {
	($($e:expr),*) => { Vec::from(vec![$($e),*]) };
	($($e:expr,)*) => { Vec::from(vec![$($e),*]) };
	($e:expr; $d:expr) => { Vec::from(vec![$e; $d]) };
	($e:expr; $d:expr $(; $ds:expr)+) => { Vec::from(vec![mat![$e $(; $ds)*]; $d]) };
}

fn group(title: String) -> Group {
    Group::new().add(Title::new().add(Text::new(title)))
}

const MAX_T: usize = 5000;

#[derive(Clone, Debug)]
pub struct Input {
    pub N: usize,
    pub M: usize,
    pub eps: f64,
    pub delta: f64,
    pub s: (i64, i64),
    pub ps: Vec<(i64, i64)>,
    pub walls: Vec<(i64, i64, i64, i64)>,
    pub fs: Vec<(i64, i64)>,
    pub alphas: Vec<f64>,
}

pub fn parse_input(f: &str) -> Input {
    let f = proconio::source::once::OnceSource::from(f);
    input! {
        from f,
        N: usize, M: usize, eps: f64, delta: f64,
        s: (i64, i64),
        ps: [(i64, i64); N],
        walls: [(i64, i64, i64, i64); M],
        alphas: [f64; MAX_T],
        fs: [(i64, i64); MAX_T],
    }
    Input {
        N,
        M,
        eps,
        delta,
        s,
        ps,
        walls,
        fs,
        alphas,
    }
}

pub fn read<T: Copy + PartialOrd + std::fmt::Display + std::str::FromStr, R: RangeBounds<T>>(
    token: Option<&str>,
    range: R,
) -> Result<T, String> {
    if let Some(v) = token {
        if let Ok(v) = v.parse::<T>() {
            if !range.contains(&v) {
                Err(format!("Out of range: {}", v))
            } else {
                Ok(v)
            }
        } else {
            Err(format!("Parse error: {}", v))
        }
    } else {
        Err("Unexpected EOF".to_owned())
    }
}

pub struct Output {
    pub out: Vec<(char, i64, i64)>,
}

pub fn parse_output(_input: &Input, f: &str) -> Result<Output, String> {
    let mut out = vec![];
    for line in f.lines() {
        if line.starts_with('#') {
            continue;
        }
        let mut it = line.split_whitespace();
        let a = read(it.next(), 'A'..'Z')?;
        let x = read(it.next(), -100000..=100000)?;
        let y = read(it.next(), -100000..=100000)?;
        if a != 'A' && a != 'S' {
            return Err(format!("Invalid action: {}", a));
        } else if a == 'A' && x * x + y * y > 500 * 500 {
            return Err(format!("Out of range: ({}, {})", x, y));
        } else if a == 'S' && x * x + y * y > 10000000000 {
            return Err(format!("Out of range: ({}, {})", x, y));
        } else if a == 'S' && (x, y) == (0, 0) {
            return Err(format!("Out of range: ({}, {})", x, y));
        }
        out.push((a, x, y));
    }
    if out.len() > MAX_T {
        return Err(format!("Too many actions: {}", out.len()));
    }
    Ok(Output { out })
}

use std::ops::*;
#[derive(Clone, Copy, Default, Debug, PartialEq, PartialOrd)]
pub struct P(pub f64, pub f64);

impl Add for P {
    type Output = P;
    fn add(self, a: P) -> P {
        P(self.0 + a.0, self.1 + a.1)
    }
}

impl Sub for P {
    type Output = P;
    fn sub(self, a: P) -> P {
        P(self.0 - a.0, self.1 - a.1)
    }
}

impl Mul<f64> for P {
    type Output = P;
    fn mul(self, a: f64) -> P {
        P(self.0 * a, self.1 * a)
    }
}

impl P {
    pub fn dot(self, a: P) -> f64 {
        (self.0 * a.0) + (self.1 * a.1)
    }
    pub fn det(self, a: P) -> f64 {
        (self.0 * a.1) - (self.1 * a.0)
    }
    pub fn abs2(self) -> f64 {
        self.dot(self)
    }
}

fn sig<T>(x: T) -> i32
where
    T: Default + PartialOrd,
{
    match x.partial_cmp(&T::default()) {
        Some(Ordering::Greater) => 1,
        Some(Ordering::Less) => -1,
        _ => 0,
    }
}

impl P {
    pub fn dist2_sp((p1, p2): (P, P), q: P) -> f64 {
        if (p2 - p1).dot(q - p1) <= 0.0 {
            (q - p1).abs2()
        } else if (p1 - p2).dot(q - p2) <= 0.0 {
            (q - p2).abs2()
        } else {
            P::dist2_lp((p1, p2), q)
        }
    }
    pub fn dist2_lp((p1, p2): (P, P), q: P) -> f64 {
        let det = (p2 - p1).det(q - p1);
        det * det / (p2 - p1).abs2()
    }
    pub fn crs_sp((p1, p2): (P, P), q: P) -> bool {
        P::crs_lp((p1, p2), q) && (q - p1).dot(q - p2) <= 0.0
    }
    pub fn crs_lp((p1, p2): (P, P), q: P) -> bool {
        (p2 - p1).det(q - p1) == 0.0
    }
    pub fn crs_ss((p1, p2): (P, P), (q1, q2): (P, P)) -> bool {
        let sort = |a, b| {
            if a < b {
                (a, b)
            } else {
                (b, a)
            }
        };
        let (lp0, up0) = sort(p1.0, p2.0);
        let (lq0, uq0) = sort(q1.0, q2.0);
        let (lp1, up1) = sort(p1.1, p2.1);
        let (lq1, uq1) = sort(q1.1, q2.1);
        if up0 < lq0 || uq0 < lp0 || up1 < lq1 || uq1 < lp1 {
            return false;
        }
        return sig((p2 - p1).det(q1 - p1)) * sig((p2 - p1).det(q2 - p1)) <= 0
            && sig((q2 - q1).det(p1 - q1)) * sig((q2 - q1).det(p2 - q1)) <= 0;
    }
    pub fn pi_ll((p1, p2): (P, P), (q1, q2): (P, P)) -> Option<P> {
        let d = (q2 - q1).det(p2 - p1);
        if d == 0.0 {
            return None;
        }
        let r = p1 * d + (p2 - p1) * (q2 - q1).det(q1 - p1);
        Some(P(r.0 / d, r.1 / d))
    }
}

struct Sim {
    visited: Vec<bool>,
    score: i64,
    crt_score: i64,
    p: P,
    v: P,
    t: usize,
}

impl Sim {
    fn new(input: &Input) -> Self {
        let visited = vec![false; input.ps.len()];
        let score = 0;
        let crt_score = 0;
        let p = P(input.s.0 as f64, input.s.1 as f64);
        let v = P(0.0, 0.0);
        Self {
            visited,
            score,
            crt_score,
            p,
            v,
            t: 0,
        }
    }
    fn query(&mut self, input: &Input, mv: char, x: i64, y: i64) -> (i32, Vec<usize>, i64) {
        let mut ret = -1;
        match mv {
            'A' => {
                self.v = self.v + P(x as f64, y as f64);
            }
            'S' => {
                let mut d = 1e9;
                for wall in input.walls.iter().chain(
                    [
                        (-100000, -100000, -100000, 100000),
                        (-100000, 100000, 100000, 100000),
                        (100000, 100000, 100000, -100000),
                        (100000, -100000, -100000, -100000),
                    ]
                    .iter(),
                ) {
                    let dir = P(x as f64, y as f64);
                    let w1 = P(wall.0 as f64, wall.1 as f64);
                    let w2 = P(wall.2 as f64, wall.3 as f64);
                    if let Some(p) = P::pi_ll((self.p, self.p + dir), (w1, w2)) {
                        if sig(dir.det(w1 - self.p)) * sig(dir.det(w2 - self.p)) <= 0
                            && (p - self.p).dot(dir) >= 0.0
                        {
                            d.setmin((p - self.p).abs2().sqrt());
                        }
                    }
                }
                d *= input.alphas[self.t];
                ret = d.round() as i64;
            }
            _ => {
                unreachable!()
            }
        }
        self.v = self.v + P(input.fs[self.t].0 as f64, input.fs[self.t].1 as f64);
        self.crt_score -= 2;
        self.t += 1;
        let q = self.p + self.v;
        if q.0 < -100000.0
            || 100000.0 < q.0
            || q.1 < -100000.0
            || 100000.0 < q.1
            || input.walls.iter().any(|&(x1, y1, x2, y2)| {
                P::crs_ss(
                    (P(x1 as f64, y1 as f64), P(x2 as f64, y2 as f64)),
                    (self.p, q),
                )
            })
        {
            self.crt_score -= 100;
            self.v = P(0.0, 0.0);
            return (1, vec![], ret);
        } else {
            let mut hit = vec![];
            for i in 0..input.ps.len() {
                if !self.visited[i]
                    && P::dist2_sp((self.p, q), P(input.ps[i].0 as f64, input.ps[i].1 as f64))
                        <= 1000000.0
                {
                    self.visited[i] = true;
                    self.crt_score += 1000;
                    hit.push(i);
                }
            }
            self.p = q;
            self.score.setmax(self.crt_score);
            (0, hit, ret)
        }
    }
}

#[wasm_bindgen]
pub struct Res {
    pub score: i32,
    #[wasm_bindgen(getter_with_clone)]
    pub error: String,
    #[wasm_bindgen(getter_with_clone)]
    pub svg: String,
}

#[wasm_bindgen]
pub fn vis(input: &str, output: &str, t: i32) -> Res {
    let input = parse_input(input);
    let output = parse_output(&input, output).unwrap();
    let t = min(t as usize + 1, output.out.len());

    let mut sim = Sim::new(&input);
    let mut err = "".to_string();
    let mut sp = (0, 0);
    for turn in 0..t {
        if turn == t - 1 {
            sp = (sim.p.0.round() as i64, sim.p.1.round() as i64);
        }
        let (mv, x, y) = output.out[turn];
        sim.query(&input, mv, x, y);
    }
    let ep = (sim.p.0.round() as i64, sim.p.1.round() as i64);
    let max_px = 650usize;
    let to_pos = |p: i64| {
        let p = p + 100000;
        p as f64 * 650.0 / 200000.0
    };
    let mut doc = svg::Document::new()
        .set("viewBox", (-5, -5, max_px + 10, max_px + 10))
        .set("width", max_px + 10)
        .set("height", max_px + 10);
    doc = doc.add(
        Rectangle::new()
            .set("x", 0)
            .set("y", 0)
            .set("height", max_px)
            .set("width", max_px)
            .set("fill", "#eee")
            .set("stroke", "black")
            .set("stroke-width", 1),
    );
    doc = doc.add(
        Line::new()
            .set("x1", to_pos(sp.0))
            .set("y1", to_pos(sp.1))
            .set("x2", to_pos(ep.0))
            .set("y2", to_pos(ep.1))
            .set("stroke", "red"),
    );
    let mut grp = group(format!(
        "pos:{} {}\nv:{} {}",
        ep.0,
        ep.1,
        sim.v.0.round() as i64,
        sim.v.1.round() as i64
    ));
    grp = grp.add(
        Rectangle::new()
            .set("x", to_pos(ep.0) - 2.5)
            .set("y", to_pos(ep.1) - 2.5)
            .set("height", 5)
            .set("width", 5)
            .set("fill", "blue")
            .set("stroke", "black")
            .set("stroke-width", 1),
    );
    doc = doc.add(grp);

    for wall_id in 0..input.M {
        doc = doc.add(
            Line::new()
                .set("x1", to_pos(input.walls[wall_id].0))
                .set("y1", to_pos(input.walls[wall_id].1))
                .set("x2", to_pos(input.walls[wall_id].2))
                .set("y2", to_pos(input.walls[wall_id].3))
                .set("stroke", "black"),
        );
    }
    for target_id in 0..input.N {
        let color = if sim.visited[target_id] {
            "red"
        } else {
            "yellow"
        };
        doc = doc.add(
            Circle::new()
                .set("cx", to_pos(input.ps[target_id].0))
                .set("cy", to_pos(input.ps[target_id].1))
                .set("r", 3.25)
                .set("fill", color)
                .set("stroke", "black")
                .set("stroke-width", 1),
        );
        let mut grp = group(format!(
            "id:{}\npos:{} {}",
            target_id, input.ps[target_id].0, input.ps[target_id].1
        ));

        grp = grp.add(
            Circle::new()
                .set("cx", to_pos(input.ps[target_id].0))
                .set("cy", to_pos(input.ps[target_id].1))
                .set("r", 3.25)
                .set("fill", color)
                .set("stroke", "black")
                .set("stroke-width", 1),
        );
        doc = doc.add(grp);
    }

    Res {
        score: sim.score as i32,
        error: err,
        svg: doc.to_string(),
    }
}

#[wasm_bindgen]
pub fn get_max_turn(input: &str, output: &str) -> i32 {
    let input = parse_input(input);
    let output = parse_output(&input, output).unwrap();

    output.out.len() as i32
    /*let mut state = State::new();
    for t in 0..T {
        match state.advance() {
            Ok(_) => {}
            Err(_errmsg) => {
                return t as i32;
            }
        };
        state.add_enemies(&input, t);
        match state.walk(&output, t) {
            Ok(_) => {}
            Err(_errmsg) => {
                return t as i32;
            }
        }
        state.attack();
    }
    T as i32*/
}
