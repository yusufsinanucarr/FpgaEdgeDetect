// sobel_filter.v
// Sobel Edge Detection Core with Runtime Adjustable Threshold
module sobel_filter #(
    parameter DEFAULT_THRESHOLD = 80
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [7:0]  threshold_val, // Runtime adjustable threshold

    // 3x3 Window Inputs
    input  wire [7:0]  p00, input  wire [7:0]  p01, input  wire [7:0]  p02,
    input  wire [7:0]  p10, input  wire [7:0]  p11, input  wire [7:0]  p12,
    input  wire [7:0]  p20, input  wire [7:0]  p21, input  wire [7:0]  p22,
    input  wire        window_valid,
    input  wire        is_border,

    output reg  [7:0]  edge_pixel,
    output reg         sobel_valid
);

    // Intermediate 11-bit sums (Max value: 255 + 510 + 255 = 1020)
    wire [10:0] gx_pos = {3'b0, p02} + ({2'b0, p12, 1'b0}) + {3'b0, p22};
    wire [10:0] gx_neg = {3'b0, p00} + ({2'b0, p10, 1'b0}) + {3'b0, p20};

    wire [10:0] gy_pos = {3'b0, p00} + ({2'b0, p01, 1'b0}) + {3'b0, p02};
    wire [10:0] gy_neg = {3'b0, p20} + ({2'b0, p21, 1'b0}) + {3'b0, p22};

    // Absolute differences |Gx| and |Gy|
    wire [10:0] abs_gx = (gx_pos >= gx_neg) ? (gx_pos - gx_neg) : (gx_neg - gx_pos);
    wire [10:0] abs_gy = (gy_pos >= gy_neg) ? (gy_pos - gy_neg) : (gy_neg - gy_pos);

    // Total gradient magnitude
    wire [11:0] grad_sum = {1'b0, abs_gx} + {1'b0, abs_gy};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            edge_pixel  <= 8'd0;
            sobel_valid <= 1'b0;
        end else begin
            sobel_valid <= window_valid;

            if (window_valid) begin
                if (is_border) begin
                    edge_pixel <= 8'h00; // Black border
                end else if (grad_sum > {4'b0, threshold_val}) begin
                    edge_pixel <= 8'hFF; // White edge
                end else begin
                    edge_pixel <= 8'h00; // Black non-edge
                end
            end
        end
    end

endmodule
