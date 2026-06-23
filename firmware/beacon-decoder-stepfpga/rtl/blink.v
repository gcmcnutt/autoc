// F1 visual + telemetry proof: ripple the 8 user LEDs AND emit the LED byte over UART each time it changes
// ("the blink value every time it updates"). Proves code → FPGA → USB-serial → host. The correlator RTL
// replaces this once the loop is proven; the UART block is reused for the real telemetry (F3/F4).
//
// UART line per change: <HH>\r\n  (LED byte as 2 hex chars). Read raw on the host (beacon-telemetry --raw).
module blink (input clk, output [7:0] LEDs, output uart);
  // free-running counter → visible binary ripple on the 8 LEDs
  reg [31:0] cnt = 32'd0;
  always @(posedge clk) cnt <= cnt + 1'b1;
  wire [7:0] ledval = cnt[27:20];
  assign LEDs = ledval;

  // 4-bit nibble → ASCII hex
  function [7:0] hex(input [3:0] n); hex = (n < 4'd10) ? (8'h30 + n) : (8'h41 + n - 8'd10); endfunction

  // UART transmitter (12 MHz / 115200)
  reg        send = 1'b0;
  reg  [7:0] txd  = 8'd0;
  wire       busy;
  uart_tx #(.CLKS_PER_BIT(104)) u (.clk(clk), .send(send), .data(txd), .tx(uart), .busy(busy));

  // On each change of the LED byte, push the 4-byte message [hexHi, hexLo, CR, LF] through the UART.
  reg [7:0] last = 8'd0;     // last value for which a message was started (advances only at msg start)
  reg [1:0] mi   = 2'd0;
  reg       sending = 1'b0;

  always @(posedge clk) begin
    send <= 1'b0;
    if (!sending) begin
      if (ledval != last) begin last <= ledval; mi <= 2'd0; sending <= 1'b1; end
    end else if (send) begin
      // one idle cycle after a pulse so uart_tx can raise busy
    end else if (!busy) begin
      case (mi)
        2'd0: txd <= hex(last[7:4]);
        2'd1: txd <= hex(last[3:0]);
        2'd2: txd <= 8'h0D;        // CR
        2'd3: txd <= 8'h0A;        // LF
      endcase
      send <= 1'b1;
      if (mi == 2'd3) sending <= 1'b0;
      else            mi <= mi + 1'b1;
    end
  end
endmodule
