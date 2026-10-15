// gaussian_filter.v
// 3x3 Gaussian Blur Filter for Noise Reduction
// Kernel:
// [ 1  2  1 ]
// [ 2  4  2 ] / 16
// [ 1  2  1 ]
module gaussian_filter (
    input  wire        clk,
    input  wire        rst_n,

    // 3x3 Window Inputs
    input  wire [7:0]  p00, input  wire [7:0]  p01, input  wire [7:0]  p02,
    input  wire [7:0]  p10, input  wire [7:0]  p11, input  wire [7:0]  p12,
    input  wire [7:0]  p20, input  wire [7:0]  p21, input  wire [7:0]  p22,
    input  wire        window_valid,
    input  wire        is_border,

    output reg  [7:0]  smooth_pixel,
    output reg         smooth_valid
);

    // Weighted sum (Maximum value: 16 * 255 = 4080 -> fits in 12 bits)
    wire [11:0] sum =
        {4'd0, p00}               + {3'd0, p01, 1'b0}        + {4'd0, p02} +
        {3'd0, p10, 1'b0}        + {2'd0, p11, 2'b00}       + {3'd0, p12, 1'b0} +
        {4'd0, p20}               + {3'd0, p21, 1'b0}        + {4'd0, p22};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            smooth_pixel <= 8'd0;
            smooth_valid <= 1'b0;
        end else begin
            smooth_valid <= window_valid;

            if (window_valid) begin
                if (is_border) begin
                    smooth_pixel <= 8'd0; // Clean border
                end else begin
                    smooth_pixel <= sum[11:4]; // Bit shift right by 4 (divide by 16)
                end
            end
        end
    end

endmodule
