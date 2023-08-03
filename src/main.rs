#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")] // hide console window on Windows in release

use egui::plot::{Plot, Points};

const G: f64 = 6.67e-11_f64;
const DELTA_TIME: f64 = 60.0 * 60.0 * 24.0;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let body_sim = BodySim::new([
        Body::new(2.0e30, (0.0, 0.0), (0.0, 0.0)),
        Body::new(6.0e24, (0.0, 1.5e11), (30_000.0, 0.0))
    ]);

    let options = eframe::NativeOptions {
        initial_window_size: Some(egui::vec2(1000.0, 800.0)),
        ..Default::default()
    };
    eframe::run_native(
        "N-Body",
        options,
        Box::new(|_cc| Box::new(NBody::new(body_sim))),
    );

    Ok(())
}

struct BodySim {
    bodies: [Body; 2]
}

impl BodySim {

    fn new(bodies: [Body; 2]) -> BodySim {
        BodySim { bodies }
    }

    fn sim_step(&mut self) {
        let mut bodies_clone = self.bodies.clone();
        for i in 0..self.bodies.len() {
            for j in 0..self.bodies.len() {
                if i == j { continue; }
                let acc: (f64, f64) = {
                    let local = &self.bodies[i];
                    let other = &self.bodies[j];
                    let dist_x = other.pos.0 - local.pos.0;
                    let dist_y = other.pos.1 - local.pos.1;
                    let dist = (dist_x * dist_x + dist_y * dist_y).sqrt();
                    if dist == 0.0 {
                        (0.0, 0.0)
                    } else {
                        let acc = G * other.mass as f64 / (dist * dist);
                        let acc_x = acc * dist_x / dist;
                        let acc_y = acc * dist_y / dist;
                        (acc_x, acc_y)
                    }
                };
                bodies_clone[i].vel = (bodies_clone[i].vel.0 + acc.0 * DELTA_TIME, bodies_clone[i].vel.1 + acc.1 * DELTA_TIME);
                bodies_clone[i].pos = (bodies_clone[i].pos.0 + bodies_clone[i].vel.0 * DELTA_TIME, bodies_clone[i].pos.1 + bodies_clone[i].vel.1 * DELTA_TIME);
            }
        }
        self.bodies = bodies_clone;
    }

    fn to_points(&self) -> Points {
        let mapped = self.bodies.iter().map(|b| [b.pos.0, b.pos.1]).collect::<Vec<[f64; 2]>>();
        Points::new(mapped).radius(5.0)
    }
}

#[derive(Clone)]
struct Body {
    mass: f64,
    pos: (f64, f64),
    vel: (f64, f64),
}

impl Body {
    fn new(mass: f64, pos: (f64, f64), vel: (f64, f64)) -> Body {
        Body { mass, pos, vel }
    }
}

struct NBody {
    body_sim: BodySim
}

impl NBody {
    fn new(body_sim: BodySim) -> Self {
        Self { body_sim }
    }
}

impl eframe::App for NBody {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            let markers_plot = Plot::new("nbody")
                .data_aspect(1.0).include_x(1.5e11 * 4.0).include_x(1.5e11 * -4.0).include_y(1.5e11 * 4.0).include_y(1.5e11 * -4.0);
            markers_plot.show(ui, |plot_ui| {
                plot_ui.points(self.body_sim.to_points());
            })
        });
        self.body_sim.sim_step();
        ctx.request_repaint();
    }
}
