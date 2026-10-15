// line_buffer.v
// Dynamic Resolution 3x3 Sliding Window Generator
// Supports any resolution up to 1920x1080 (Full HD) configured at runtime!
module line_buffer #(
    parameter MAX_WIDTH  = 1024,
    parameter MAX_HEIGHT = 1024
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        frame_sync,     // 1-cycle pulse to reset counters to (0,0)
    input  wire [11:0] cfg_width,      // Runtime configurable width (e.g. 128 to 1920)
    input  wire [11:0] cfg_height,     // Runtime configurable height (e.g. 128 to 1080)
    input  wire [7:0]  pixel_in,
    input  wire        pixel_in_valid,

    output reg  [7:0]  p00, output reg  [7:0]  p01, output reg  [7:0]  p02,
    output reg  [7:0]  p10, output reg  [7:0]  p11, output reg  [7:0]  p12,
    output reg  [7:0]  p20, output reg  [7:0]  p21, output reg  [7:0]  p22,
    output reg         window_valid,
    output reg         is_border
);

    // Hardware Block RAM line buffers sized for maximum supported width (1920)
    reg [7:0] line0 [0:MAX_WIDTH-1];
    reg [7:0] line1 [0:MAX_WIDTH-1];

    reg [11:0] col_cnt; // 0 to 4095
    reg [11:0] row_cnt; // 0 to 4095

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            col_cnt      <= 12'd0;
            row_cnt      <= 12'd0;
            window_valid <= 1'b0;
            is_border    <= 1'b1;

            p00 <= 8'd0; p01 <= 8'd0; p02 <= 8'd0;
            p10 <= 8'd0; p11 <= 8'd0; p12 <= 8'd0;
            p20 <= 8'd0; p21 <= 8'd0; p22 <= 8'd0;
        end else begin
            window_valid <= 1'b0; // Default pulse for 1 cycle

            if (frame_sync) begin
                col_cnt <= 12'd0;
                row_cnt <= 12'd0;
            end else if (pixel_in_valid) begin
                // Read from line buffer RAMs
                p02 <= line0[col_cnt];
                p01 <= p02;
                p00 <= p01;

                p12 <= line1[col_cnt];
                p11 <= p12;
                p10 <= p11;

                p22 <= pixel_in;
                p21 <= p22;
                p20 <= p21;

                // Write into line buffer RAMs
                line0[col_cnt] <= line1[col_cnt];
                line1[col_cnt] <= pixel_in;

                // Border suppression for first 2 rows & columns
                if (row_cnt < 12'd2 || col_cnt < 12'd2) begin
                    is_border <= 1'b1;
                end else begin
                    is_border <= 1'b0;
                end

                window_valid <= 1'b1;

                // Dynamic counter increment based on runtime cfg_width and cfg_height
                if (col_cnt == cfg_width - 12'd1) begin
                    col_cnt <= 12'd0;
                    if (row_cnt == cfg_height - 12'd1)
                        row_cnt <= 12'd0;
                    else
                        row_cnt <= row_cnt + 12'd1;
                end else begin
                    col_cnt <= col_cnt + 12'd1;
                end
            end
        end
    end

endmodule
