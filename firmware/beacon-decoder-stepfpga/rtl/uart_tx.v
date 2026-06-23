// Minimal 8N1 UART transmitter. CLKS_PER_BIT = clk_hz / baud (12 MHz / 115200 = 104).
// Pulse `send` (1 clk) with `data` valid; `busy` high until the stop bit completes. Idle line = high.
module uart_tx #(parameter CLKS_PER_BIT = 104) (
  input        clk,
  input        send,
  input  [7:0] data,
  output reg   tx   = 1'b1,
  output reg   busy = 1'b0
);
  localparam IDLE = 2'd0, START = 2'd1, DATA = 2'd2, STOP = 2'd3;
  reg [1:0]  state = IDLE;
  reg [15:0] clkcnt = 0;
  reg [2:0]  bitidx = 0;
  reg [7:0]  sh = 0;

  always @(posedge clk) begin
    case (state)
      IDLE: begin
        tx <= 1'b1; busy <= 1'b0; clkcnt <= 0; bitidx <= 0;
        if (send) begin sh <= data; busy <= 1'b1; state <= START; end
      end
      START: begin
        tx <= 1'b0;
        if (clkcnt < CLKS_PER_BIT-1) clkcnt <= clkcnt + 1'b1;
        else begin clkcnt <= 0; state <= DATA; end
      end
      DATA: begin
        tx <= sh[bitidx];
        if (clkcnt < CLKS_PER_BIT-1) clkcnt <= clkcnt + 1'b1;
        else begin
          clkcnt <= 0;
          if (bitidx < 3'd7) bitidx <= bitidx + 1'b1;
          else begin bitidx <= 0; state <= STOP; end
        end
      end
      STOP: begin
        tx <= 1'b1;
        if (clkcnt < CLKS_PER_BIT-1) clkcnt <= clkcnt + 1'b1;
        else begin clkcnt <= 0; busy <= 1'b0; state <= IDLE; end
      end
    endcase
  end
endmodule
