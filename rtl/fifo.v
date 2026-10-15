// fifo.v
// Synchronous FIFO Buffer for UART Transmission
module fifo #(
    parameter DATA_WIDTH = 8,
    parameter ADDR_WIDTH = 8 // 256 elements
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire [DATA_WIDTH-1:0] din,
    input  wire                  push,
    input  wire                  pop,
    output reg  [DATA_WIDTH-1:0] dout,
    output wire                  empty,
    output wire                  full
);

    localparam DEPTH = 1 << ADDR_WIDTH;

    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    reg [ADDR_WIDTH-1:0] wr_ptr;
    reg [ADDR_WIDTH-1:0] rd_ptr;
    reg [ADDR_WIDTH:0]   count;

    assign empty = (count == 0);
    assign full  = (count == DEPTH);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= {ADDR_WIDTH{1'b0}};
            rd_ptr <= {ADDR_WIDTH{1'b0}};
            count  <= {(ADDR_WIDTH+1){1'b0}};
            dout   <= {DATA_WIDTH{1'b0}};
        end else begin
            // Write operation
            if (push && !full) begin
                mem[wr_ptr] <= din;
                wr_ptr      <= wr_ptr + 1'b1;
            end

            // Read operation
            if (pop && !empty) begin
                dout   <= mem[rd_ptr];
                rd_ptr <= rd_ptr + 1'b1;
            end

            // Counter update
            case ({push && !full, pop && !empty})
                2'b10: count <= count + 1'b1;
                2'b01: count <= count - 1'b1;
                default: count <= count;
            endcase
        end
    end

endmodule
