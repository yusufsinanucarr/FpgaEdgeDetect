// uart_tx.v
// UART Transmitter for 27 MHz Clock, 115200 Baud (8N1)
module uart_tx #(
    parameter CLKS_PER_BIT = 234 // 27,000,000 / 115,200 = 234
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] tx_data,
    input  wire       tx_start,
    output reg        tx_pin,
    output reg        tx_busy
);

    localparam STATE_IDLE      = 3'b000;
    localparam STATE_START_BIT = 3'b001;
    localparam STATE_DATA_BITS = 3'b010;
    localparam STATE_STOP_BIT  = 3'b011;
    localparam STATE_CLEANUP   = 3'b100;

    reg [2:0]  state;
    reg [15:0] clk_count;
    reg [2:0]  bit_index;
    reg [7:0]  tx_data_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= STATE_IDLE;
            clk_count   <= 16'd0;
            bit_index   <= 3'd0;
            tx_data_reg <= 8'd0;
            tx_pin      <= 1'b1; // Idle high
            tx_busy     <= 1'b0;
        end else begin
            case (state)
                STATE_IDLE: begin
                    tx_pin    <= 1'b1;
                    clk_count <= 16'd0;
                    bit_index <= 3'd0;

                    if (tx_start) begin
                        tx_busy     <= 1'b1;
                        tx_data_reg <= tx_data;
                        state       <= STATE_START_BIT;
                    end else begin
                        tx_busy     <= 1'b0;
                    end
                end

                STATE_START_BIT: begin
                    tx_pin <= 1'b0; // Start bit is low

                    if (clk_count < (CLKS_PER_BIT - 1)) begin
                        clk_count <= clk_count + 16'd1;
                    end else begin
                        clk_count <= 16'd0;
                        state     <= STATE_DATA_BITS;
                    end
                end

                STATE_DATA_BITS: begin
                    tx_pin <= tx_data_reg[bit_index];

                    if (clk_count < (CLKS_PER_BIT - 1)) begin
                        clk_count <= clk_count + 16'd1;
                    end else begin
                        clk_count <= 16'd0;
                        if (bit_index < 3'd7) begin
                            bit_index <= bit_index + 3'd1;
                        end else begin
                            bit_index <= 3'd0;
                            state     <= STATE_STOP_BIT;
                        end
                    end
                end

                STATE_STOP_BIT: begin
                    tx_pin <= 1'b1; // Stop bit is high

                    if (clk_count < (CLKS_PER_BIT - 1)) begin
                        clk_count <= clk_count + 16'd1;
                    end else begin
                        clk_count <= 16'd0;
                        state     <= STATE_CLEANUP;
                    end
                end

                STATE_CLEANUP: begin
                    tx_busy <= 1'b0;
                    state   <= STATE_IDLE;
                end

                default: state <= STATE_IDLE;
            endcase
        end
    end

endmodule
