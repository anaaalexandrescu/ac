use std::error::Error;
use std::fs;
use std::path::PathBuf;

use lidar_clustering_hdl::segmentation_step;
use rhdl::prelude::*;

fn parse_bit_widths(signature: &str) -> Result<Vec<usize>, Box<dyn Error>> {
    let mut widths = Vec::new();
    let mut rest = signature;

    while let Some(start) = rest.find('[') {
        let after = &rest[start + 1..];
        let end = after.find(']').ok_or("missing ']' in signature")?;
        let range = &after[..end];
        if let Some((msb, lsb)) = range.split_once(':') {
            let msb: usize = msb.trim().parse()?;
            let lsb: usize = lsb.trim().parse()?;
            let width = msb.saturating_sub(lsb) + 1;
            widths.push(width);
        }
        rest = &after[end + 1..];
    }

    Ok(widths)
}

fn main() -> Result<(), Box<dyn Error>> {
    let out_dir = std::env::args().nth(1).unwrap_or_else(|| "synth".to_string());
    fs::create_dir_all(&out_dir)?;

    let kernel = compile_design::<segmentation_step>(CompilationMode::Synchronous)?;
    let vlog = kernel.as_vlog()?;
    let vlog_str = vlog.pretty();

    let out_path = PathBuf::from(&out_dir).join("segmentation_step.v");
    fs::write(&out_path, vlog_str)?;
    println!("Wrote {}", out_path.display());

    let signature = fs::read_to_string(&out_path)?
        .lines()
        .next()
        .ok_or("empty verilog output")?
        .to_string();
    let widths = parse_bit_widths(&signature)?;
    if widths.len() < 4 {
        return Err("unexpected kernel signature width count".into());
    }
    let out_w = widths[0];
    let in0_w = widths[1];
    let in1_w = widths[2];
    let in2_w = widths[3];

    let wrapper = format!(
        "module segmentation_step_top(\n\
    input wire clk,\n\
    input wire rst,\n\
    input wire [{in0_msb}:0] core_in,\n\
    input wire [{in1_msb}:0] point_in,\n\
    input wire [{in2_msb}:0] state_in,\n\
    output reg [{out_msb}:0] out\n\
);\n\
    reg [{in0_msb}:0] core_reg;\n\
    reg [{in1_msb}:0] point_reg;\n\
    reg [{in2_msb}:0] state_reg;\n\
    wire [{out_msb}:0] comb_out;\n\
\n\
    assign comb_out = kernel_segmentation_step(core_reg, point_reg, state_reg);\n\
\n\
    always @(posedge clk) begin\n\
        if (rst) begin\n\
            core_reg <= {{{in0_w}{{1'b0}}}};\n\
            point_reg <= {{{in1_w}{{1'b0}}}};\n\
            state_reg <= {{{in2_w}{{1'b0}}}};\n\
            out <= {{{out_w}{{1'b0}}}};\n\
        end else begin\n\
            core_reg <= core_in;\n\
            point_reg <= point_in;\n\
            state_reg <= state_in;\n\
            out <= comb_out;\n\
        end\n\
    end\n\
\n\
{function}\n\
endmodule\n",
        in0_msb = in0_w - 1,
        in1_msb = in1_w - 1,
        in2_msb = in2_w - 1,
        out_msb = out_w - 1,
        in0_w = in0_w,
        in1_w = in1_w,
        in2_w = in2_w,
        out_w = out_w,
        function = fs::read_to_string(&out_path)?,
    );

    let wrapper_path = PathBuf::from(&out_dir).join("segmentation_step_top.v");
    fs::write(&wrapper_path, wrapper)?;
    println!("Wrote {}", wrapper_path.display());
    Ok(())
}
