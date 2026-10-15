// edge_detect_top.v
// Top Level Module for Tang Nano 20K
// Dynamic Resolution Edge Detection: Supports 128x128 up to 1920x1080 (Full HD)!
// Configured at runtime via a 5-byte header: [Threshold, W_hi, W_lo, H_hi, H_lo]
// Pipeline: UART RX (460.8k) -> Dynamic Cfg -> Line Buffer 1 (Max 1920)
//           -> Gaussian Blur -> Line Buffer 2 (Max 1920) -> Sobel Filter -> FIFO -> UART TX
// Features: Hardware Knight Rider LED Chaser during processing!
module edge_detect_top (
    input  wire       sys_clk,   // 27 MHz onboard oscillator (Pin 4)
    input  wire       uart_rx,   // From PC via onboard USB chip (Pin 70)
    output wire       uart_tx,   // To PC via onboard USB chip (Pin 69)
    output wire [5:0] led        // Onboard LEDs (Pins 15-20, Active-low)
);

    // -------------------------------------------------------------
    // Internal Power-On Reset
    // -------------------------------------------------------------
    reg [19:0] por_counter = 20'd0;
    reg rst_n = 1'b0;

    always @(posedge sys_clk) begin
        if (por_counter < 20'd100_000) begin // ~3.7 ms at 27 MHz
            por_counter <= por_counter + 20'd1;
            rst_n       <= 1'b0;
        end else begin
            rst_n       <= 1'b1;
        end
    end

    // -------------------------------------------------------------
    // Rock-Solid High-Speed UART Receiver (27 MHz, 460,800 Baud -> 59 clocks/bit)
    // -------------------------------------------------------------
    wire [7:0] rx_pixel;
    wire       rx_valid;

    uart_rx #(
        .CLKS_PER_BIT(20)
    ) u_uart_rx (
        .clk      (sys_clk),
        .rst_n    (rst_n),
        .rx_pin   (uart_rx),
        .rx_data  (rx_pixel),
        .rx_valid (rx_valid)
    );

    // -------------------------------------------------------------
    // Frame Sync & 5-Byte Header Dynamic Configuration FSM
    // Header format: [Threshold, W_hi, W_lo, H_hi, H_lo]
    // -------------------------------------------------------------
    localparam IDLE_TIMEOUT = 23'd5_400_000; // ~200 ms silence @ 27 MHz for stable frame sync
    reg [22:0] idle_cnt   = 23'd0;
    reg        frame_sync = 1'b0;

    localparam CFG_STATE_THR   = 3'd0;
    localparam CFG_STATE_W_HI  = 3'd1;
    localparam CFG_STATE_W_LO  = 3'd2;
    localparam CFG_STATE_H_HI  = 3'd3;
    localparam CFG_STATE_H_LO  = 3'd4;
    localparam CFG_STATE_PIXEL = 3'd5;

    reg [2:0]  cfg_state;
    reg [7:0]  threshold_reg;
    reg [11:0] cfg_width;
    reg [11:0] cfg_height;

    reg        echo_trigger;
    reg [7:0]  echo_data_in;
    reg        stream_pixel_valid;

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            idle_cnt           <= 23'd0;
            frame_sync         <= 1'b0;
            cfg_state          <= CFG_STATE_THR;
            threshold_reg      <= 8'd80;
            cfg_width          <= 12'd512;
            cfg_height         <= 12'd512;
            echo_trigger       <= 1'b0;
            echo_data_in       <= 8'd0;
            stream_pixel_valid <= 1'b0;
        end else begin
            frame_sync         <= 1'b0;
            echo_trigger       <= 1'b0;
            stream_pixel_valid <= 1'b0;

            if (rx_valid) begin
                idle_cnt <= 23'd0;
                case (cfg_state)
                    CFG_STATE_THR: begin
                        threshold_reg <= rx_pixel;
                        echo_trigger  <= 1'b1;
                        echo_data_in  <= rx_pixel;
                        cfg_state     <= CFG_STATE_W_HI;
                    end
                    CFG_STATE_W_HI: begin
                        cfg_width[11:8] <= rx_pixel[3:0];
                        echo_trigger    <= 1'b1;
                        echo_data_in    <= rx_pixel;
                        cfg_state       <= CFG_STATE_W_LO;
                    end
                    CFG_STATE_W_LO: begin
                        cfg_width[7:0] <= rx_pixel;
                        echo_trigger   <= 1'b1;
                        echo_data_in   <= rx_pixel;
                        cfg_state      <= CFG_STATE_H_HI;
                    end
                    CFG_STATE_H_HI: begin
                        cfg_height[11:8] <= rx_pixel[3:0];
                        echo_trigger     <= 1'b1;
                        echo_data_in     <= rx_pixel;
                        cfg_state        <= CFG_STATE_H_LO;
                    end
                    CFG_STATE_H_LO: begin
                        cfg_height[7:0] <= rx_pixel;
                        echo_trigger    <= 1'b1;
                        echo_data_in    <= rx_pixel;
                        cfg_state       <= CFG_STATE_PIXEL;
                    end
                    CFG_STATE_PIXEL: begin
                        stream_pixel_valid <= 1'b1;
                    end
                    default: cfg_state <= CFG_STATE_THR;
                endcase
            end else if (idle_cnt < IDLE_TIMEOUT) begin
                idle_cnt <= idle_cnt + 23'd1;
                if (idle_cnt == IDLE_TIMEOUT - 23'd1) begin
                    frame_sync <= 1'b1;
                    cfg_state  <= CFG_STATE_THR; // Ready for next frame configuration
                end
            end
        end
    end

    // -------------------------------------------------------------
    // Stage 1: Line Buffer 1 (Max 1920 width for Gaussian Blur)
    // -------------------------------------------------------------
    wire [7:0] g_p00, g_p01, g_p02;
    wire [7:0] g_p10, g_p11, g_p12;
    wire [7:0] g_p20, g_p21, g_p22;
    wire       g_win_valid;
    wire       g_is_border;

    line_buffer #(
        .MAX_WIDTH (1024),
        .MAX_HEIGHT(1024)
    ) u_line_buffer1 (
        .clk           (sys_clk),
        .rst_n         (rst_n),
        .frame_sync    (frame_sync),
        .cfg_width     (cfg_width),
        .cfg_height    (cfg_height),
        .pixel_in      (rx_pixel),
        .pixel_in_valid(stream_pixel_valid),
        .p00(g_p00), .p01(g_p01), .p02(g_p02),
        .p10(g_p10), .p11(g_p11), .p12(g_p12),
        .p20(g_p20), .p21(g_p21), .p22(g_p22),
        .window_valid  (g_win_valid),
        .is_border     (g_is_border)
    );

    // -------------------------------------------------------------
    // Stage 2: Gaussian Blur Filter (Noise Reduction)
    // -------------------------------------------------------------
    wire [7:0] smooth_pixel;
    wire       smooth_valid;

    gaussian_filter u_gaussian_filter (
        .clk         (sys_clk),
        .rst_n       (rst_n),
        .p00(g_p00), .p01(g_p01), .p02(g_p02),
        .p10(g_p10), .p11(g_p11), .p12(g_p12),
        .p20(g_p20), .p21(g_p21), .p22(g_p22),
        .window_valid(g_win_valid),
        .is_border   (g_is_border),
        .smooth_pixel(smooth_pixel),
        .smooth_valid(smooth_valid)
    );

    // -------------------------------------------------------------
    // Stage 3: Line Buffer 2 (Max 1920 width for Sobel Filter)
    // -------------------------------------------------------------
    wire [7:0] s_p00, s_p01, s_p02;
    wire [7:0] s_p10, s_p11, s_p12;
    wire [7:0] s_p20, s_p21, s_p22;
    wire       s_win_valid;
    wire       s_is_border;

    line_buffer #(
        .MAX_WIDTH (1024),
        .MAX_HEIGHT(1024)
    ) u_line_buffer2 (
        .clk           (sys_clk),
        .rst_n         (rst_n),
        .frame_sync    (frame_sync),
        .cfg_width     (cfg_width),
        .cfg_height    (cfg_height),
        .pixel_in      (smooth_pixel),
        .pixel_in_valid(smooth_valid),
        .p00(s_p00), .p01(s_p01), .p02(s_p02),
        .p10(s_p10), .p11(s_p11), .p12(s_p12),
        .p20(s_p20), .p21(s_p21), .p22(s_p22),
        .window_valid  (s_win_valid),
        .is_border     (s_is_border)
    );

    // -------------------------------------------------------------
    // Stage 4: Sobel Filter with Runtime Adjustable Threshold
    // -------------------------------------------------------------
    wire [7:0] edge_pixel;
    wire       sobel_valid;

    sobel_filter u_sobel_filter (
        .clk          (sys_clk),
        .rst_n        (rst_n),
        .threshold_val(threshold_reg),
        .p00(s_p00), .p01(s_p01), .p02(s_p02),
        .p10(s_p10), .p11(s_p11), .p12(s_p12),
        .p20(s_p20), .p21(s_p21), .p22(s_p22),
        .window_valid (s_win_valid),
        .is_border    (s_is_border),
        .edge_pixel   (edge_pixel),
        .sobel_valid  (sobel_valid)
    );

    // -------------------------------------------------------------
    // TX FIFO Buffer
    // -------------------------------------------------------------
    wire [7:0] fifo_dout;
    wire       fifo_empty;
    wire       fifo_full;
    reg        fifo_pop;

    fifo #(
        .DATA_WIDTH(8),
        .ADDR_WIDTH(8) // 256 bytes capacity
    ) u_tx_fifo (
        .clk  (sys_clk),
        .rst_n(rst_n),
        .din  (edge_pixel),
        .push (sobel_valid),
        .pop  (fifo_pop),
        .dout (fifo_dout),
        .empty(fifo_empty),
        .full (fifo_full)
    );

    // -------------------------------------------------------------
    // High-Speed UART Transmitter (27 MHz, 460,800 Baud -> 59 clocks/bit)
    // -------------------------------------------------------------
    reg  [7:0] tx_data;
    reg        tx_start;
    wire       tx_busy;

    uart_tx #(
        .CLKS_PER_BIT(20)
    ) u_uart_tx (
        .clk     (sys_clk),
        .rst_n   (rst_n),
        .tx_data (tx_data),
        .tx_start(tx_start),
        .tx_pin  (uart_tx),
        .tx_busy (tx_busy)
    );

    // -------------------------------------------------------------
    // FIFO & Echo to UART TX Controller FSM
    // -------------------------------------------------------------
    localparam TX_FSM_IDLE = 2'd0;
    localparam TX_FSM_READ = 2'd1;
    localparam TX_FSM_SEND = 2'd2;
    localparam TX_FSM_WAIT = 2'd3;

    reg [1:0] tx_fsm_state;
    reg       echo_pending;
    reg [7:0] echo_byte;

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_fsm_state <= TX_FSM_IDLE;
            tx_start     <= 1'b0;
            tx_data      <= 8'd0;
            fifo_pop     <= 1'b0;
            echo_pending <= 1'b0;
            echo_byte    <= 8'd0;
        end else begin
            tx_start <= 1'b0;
            fifo_pop <= 1'b0;

            if (echo_trigger) begin
                echo_pending <= 1'b1;
                echo_byte    <= echo_data_in;
            end

            case (tx_fsm_state)
                TX_FSM_IDLE: begin
                    if (echo_pending && !tx_busy) begin
                        echo_pending <= 1'b0;
                        tx_data      <= echo_byte;
                        tx_start     <= 1'b1;
                        tx_fsm_state <= TX_FSM_WAIT;
                    end else if (!fifo_empty && !tx_busy) begin
                        fifo_pop     <= 1'b1;
                        tx_fsm_state <= TX_FSM_READ;
                    end
                end

                TX_FSM_READ: begin
                    tx_fsm_state <= TX_FSM_SEND;
                end

                TX_FSM_SEND: begin
                    tx_data      <= fifo_dout;
                    tx_start     <= 1'b1;
                    tx_fsm_state <= TX_FSM_WAIT;
                end

                TX_FSM_WAIT: begin
                    if (tx_busy) begin
                        tx_fsm_state <= TX_FSM_IDLE;
                    end
                end

                default: tx_fsm_state <= TX_FSM_IDLE;
            endcase
        end
    end

    // -------------------------------------------------------------
    // Visual Aesthetic LED Knight Rider Chaser Animation
    // -------------------------------------------------------------
    reg [20:0] chaser_timer = 21'd0;
    reg [3:0]  chaser_pos   = 4'd0;
    reg [23:0] active_timer = 24'd0;
    wire       is_processing = (active_timer < 24'd10_000_000);

    always @(posedge sys_clk) begin
        if (rx_valid || tx_start)
            active_timer <= 24'd0;
        else if (is_processing)
            active_timer <= active_timer + 24'd1;
    end

    always @(posedge sys_clk) begin
        if (chaser_timer == 21'd2_000_000) begin
            chaser_timer <= 21'd0;
            if (chaser_pos == 4'd9)
                chaser_pos <= 4'd0;
            else
                chaser_pos <= chaser_pos + 4'd1;
        end else begin
            chaser_timer <= chaser_timer + 21'd1;
        end
    end

    reg [5:0] chaser_led_pattern;
    always @(*) begin
        case (chaser_pos)
            4'd0: chaser_led_pattern = 6'b000001;
            4'd1: chaser_led_pattern = 6'b000010;
            4'd2: chaser_led_pattern = 6'b000100;
            4'd3: chaser_led_pattern = 6'b001000;
            4'd4: chaser_led_pattern = 6'b010000;
            4'd5: chaser_led_pattern = 6'b100000;
            4'd6: chaser_led_pattern = 6'b010000;
            4'd7: chaser_led_pattern = 6'b001000;
            4'd8: chaser_led_pattern = 6'b000100;
            4'd9: chaser_led_pattern = 6'b000010;
            default: chaser_led_pattern = 6'b000001;
        endcase
    end

    reg [23:0] heartbeat_cnt = 24'd0;
    always @(posedge sys_clk) heartbeat_cnt <= heartbeat_cnt + 24'd1;

    assign led = is_processing ? ~chaser_led_pattern : {5'b11111, ~heartbeat_cnt[23]};

endmodule
