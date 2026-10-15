// uart_rx.v
// UART Receiver for 27 MHz Clock, 115200 Baud (8N1)
module uart_rx #(
    parameter CLKS_PER_BIT = 234 // 27,000,000 / 115,200 = 234
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx_pin,
    output reg  [7:0] rx_data,
    output reg        rx_valid
);

    localparam STATE_IDLE      = 3'b000;
    localparam STATE_START_BIT = 3'b001;
    localparam STATE_DATA_BITS = 3'b010;
    localparam STATE_STOP_BIT  = 3'b011;
    localparam STATE_CLEANUP   = 3'b100;

    reg [2:0]  state;
    reg [15:0] clk_count;
    reg [2:0]  bit_index;
    reg [7:0]  rx_data_reg;

    // Double-flop synchronizer to avoid metastability on RX pin
    reg rx_sync_0;
    reg rx_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync_0 <= 1'b1;
            rx_sync   <= 1'b1;
        end else begin
            rx_sync_0 <= rx_pin;
            rx_sync   <= rx_sync_0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= STATE_IDLE;
            clk_count   <= 16'd0;
            bit_index   <= 3'd0;
            rx_data_reg <= 8'd0;
            rx_data     <= 8'd0;
            rx_valid    <= 1'b0;
        end else begin
            rx_valid <= 1'b0; // Default pulse for 1 cycle

            case (state)
                STATE_IDLE: begin
                    clk_count <= 16'd0;
                    bit_index <= 3'd0;
                    if (rx_sync == 1'b0) // Detected falling edge (Start bit)
                        state <= STATE_START_BIT;
                end

                STATE_START_BIT: begin
                    // Sample at the middle of the start bit
                    if (clk_count == (CLKS_PER_BIT / 2)) begin
                        if (rx_sync == 1'b0) begin
                            clk_count <= 16'd0;
                            state     <= STATE_DATA_BITS;
                        end else begin
                            state     <= STATE_IDLE; // False glitch
                        end
                    end else begin
                        clk_count <= clk_count + 16'd1;
                    end
                end

                STATE_DATA_BITS: begin
                    if (clk_count < (CLKS_PER_BIT - 1)) begin
                        clk_count <= clk_count + 16'd1;
                    end else begin
                        clk_count <= 16'd0;
                        rx_data_reg[bit_index] <= rx_sync; // Sample data bit

                        if (bit_index < 3'd7) begin
                            bit_index <= bit_index + 3'd1;
                        end else begin
                            bit_index <= 3'd0;
                            state     <= STATE_STOP_BIT;
                        end
                    end
                end

                STATE_STOP_BIT: begin
                    if (clk_count < (CLKS_PER_BIT - 1)) begin
                        clk_count <= clk_count + 16'd1;
                    end else begin
                        rx_valid  <= 1'b1; // Byte successfully received!
                        rx_data   <= rx_data_reg;
                        clk_count <= 16'd0;
                        state     <= STATE_CLEANUP;
                    end
                end

                STATE_CLEANUP: begin
                    state <= STATE_IDLE;
                end

                default: state <= STATE_IDLE;
            endcase
        end
    end

endmodule
