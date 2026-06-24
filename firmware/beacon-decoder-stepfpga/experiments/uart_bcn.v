// BCN telemetry over the StepFPGA USB-CDC: UART TX on pad A2 -> STEPLink LPC -> COM3 (host/ reads it).
// 115200 8N1. Emits the exact host frame contract (host/beacon_telemetry/frame.py), fixed-width DECIMAL so
// the existing int()-based parser works unchanged:
//   BCN,<seq4>,<adc4>,<corrA6>,<lockA1>,<marginA1>,<corrB6>,<lockB1>,<marginB1>\n
// (RX/commands on pad A3 are a later pass.)

module uart_tx #(parameter integer DIV = 104) (        // 12 MHz / 115200 = 104
  input  wire clk, input wire [7:0] data, input wire send,
  output reg  txd, output reg busy);
  reg [3:0] st = 0; reg [7:0] cnt = 0; reg [7:0] dr = 0;
  initial begin txd = 1'b1; busy = 1'b0; end
  always @(posedge clk) begin
    case (st)
      4'd0: begin txd <= 1'b1; busy <= 1'b0;
                  if (send) begin dr <= data; busy <= 1'b1; cnt <= 0; st <= 4'd1; end end
      default: begin
        txd <= (st==4'd1) ? 1'b0 : (st==4'd10) ? 1'b1 : dr[st-4'd2];   // start / d0..d7 (LSB first) / stop
        if (cnt == DIV-1) begin cnt <= 0; st <= (st==4'd10) ? 4'd0 : st + 1'b1; end
        else cnt <= cnt + 1'b1;
      end
    endcase
  end
endmodule

// UART RX (115200 8N1) on pad A3 (STEPLink LPC). Outputs one byte + a 1-clk valid strobe; mid-bit sampled.
module uart_rx #(parameter integer DIV = 104) (
  input  wire clk, input wire rxd,
  output reg [7:0] data, output reg valid);
  reg [1:0] sy = 2'b11; wire rx = sy[1];
  reg [3:0] st = 0; reg [7:0] cnt = 0; reg [7:0] sh = 0;
  initial begin data = 0; valid = 0; end
  always @(posedge clk) begin
    sy <= {sy[0], rxd};
    valid <= 1'b0;
    case (st)
      4'd0: if (!rx) begin st <= 4'd1; cnt <= 0; end                 // falling edge = start bit
      4'd1: if (cnt == DIV/2) begin if (!rx) begin cnt<=0; st<=4'd2; end else st<=4'd0; end  // confirm mid-start
            else cnt <= cnt + 1'b1;
      default: if (cnt == DIV-1) begin
                 cnt <= 0;
                 if (st <= 4'd9) begin sh <= {rx, sh[7:1]}; st <= st + 1'b1; end   // 8 data bits, LSB first
                 else begin if (rx) begin data <= sh; valid <= 1'b1; end st <= 4'd0; end  // stop bit
               end else cnt <= cnt + 1'b1;
    endcase
  end
endmodule

module bcn_tx (
  input  wire clk, input wire tick,                    // tick = emit one frame
  input  wire [13:0] seq, input wire [11:0] adc,
  input  wire [19:0] corrA, input wire [1:0] lockA, input wire [3:0] marginA,
  input  wire [19:0] corrB, input wire [1:0] lockB, input wire [3:0] marginB,
  output wire txd);

  function [27:0] p10(input [2:0] k);                  // 10^k, k=0..6
    case (k) 3'd0:p10=1; 3'd1:p10=10; 3'd2:p10=100; 3'd3:p10=1000;
             3'd4:p10=10000; 3'd5:p10=100000; default:p10=1000000; endcase
  endfunction

  // 36-char line buffer; literals preset at config, digit slots filled per frame
  reg [7:0] lbuf [0:39];
  integer j;
  initial begin
    for (j=0;j<40;j=j+1) lbuf[j]=8'h20;
    lbuf[0]="B"; lbuf[1]="C"; lbuf[2]="N"; lbuf[3]=",";
    lbuf[8]=","; lbuf[13]=","; lbuf[20]=","; lbuf[22]=","; lbuf[24]=","; lbuf[31]=","; lbuf[33]=",";
    lbuf[35]=8'h0A;                                     // newline
  end
  localparam integer LEN = 36;

  reg [3:0]  fi = 0;                                    // field index 0..7
  reg [19:0] wv = 0;                                    // working value
  reg [2:0]  w  = 0; reg [5:0] base = 0; reg [2:0] dp = 0; reg [3:0] digit = 0;
  reg [5:0]  si = 0; reg snd = 0; reg [7:0] sb = 0;
  wire txbusy;
  uart_tx u_tx (.clk(clk), .data(sb), .send(snd), .txd(txd), .busy(txbusy));

  reg [1:0] state = 0;                                  // 0 idle, 1 load, 2 convert, 3 stream
  wire [27:0] curpow = p10(w - 3'd1 - dp);              // current power of 10 (can't bit-select a call inline)
  always @(posedge clk) begin
    snd <= 1'b0;
    case (state)
      2'd0: if (tick) begin fi <= 0; state <= 2'd1; end
      2'd1: begin                                       // load field fi
        dp <= 0; digit <= 0;
        case (fi)
          4'd0: begin wv <= {6'b0, seq};      w <= 3'd4; base <= 6'd4;  end
          4'd1: begin wv <= {8'b0, adc};      w <= 3'd4; base <= 6'd9;  end
          4'd2: begin wv <= corrA;            w <= 3'd6; base <= 6'd14; end
          4'd3: begin wv <= {18'b0, lockA};   w <= 3'd1; base <= 6'd21; end
          4'd4: begin wv <= {16'b0, marginA}; w <= 3'd1; base <= 6'd23; end
          4'd5: begin wv <= corrB;            w <= 3'd6; base <= 6'd25; end
          4'd6: begin wv <= {18'b0, lockB};   w <= 3'd1; base <= 6'd32; end
          default: begin wv <= {16'b0, marginB}; w <= 3'd1; base <= 6'd34; end
        endcase
        state <= 2'd2;
      end
      2'd2: begin                                       // MSD-first power-of-10 subtraction
        if (wv >= curpow[19:0]) begin wv <= wv - curpow[19:0]; digit <= digit + 1'b1; end
        else begin
          lbuf[base+dp] <= "0" + digit; digit <= 0;
          if (dp == w-3'd1) begin
            if (fi == 4'd7) begin si <= 0; state <= 2'd3; end
            else begin fi <= fi + 1'b1; state <= 2'd1; end
          end else dp <= dp + 1'b1;
        end
      end
      2'd3: if (!txbusy && !snd) begin                  // stream lbuf[0..LEN-1]
        if (si == LEN) state <= 2'd0;
        else begin sb <= lbuf[si]; snd <= 1'b1; si <= si + 1'b1; end
      end
    endcase
  end
endmodule
