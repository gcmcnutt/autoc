`timescale 1ns/1ps
// Behavioral stand-in for the Lattice OSCH hard oscillator — iverilog can't elaborate the real primitive.
// SIMULATION ONLY (never synthesized; Diamond uses the real OSCH). Interface matches s6.v's instantiation:
//   OSCH #(.NOM_FREQ("53.2")) osc (.STDBY(1'b0), .OSC(oclk), .SEDSTDBY());
// NOM_FREQ "53.2" MHz -> ~18.8 ns period -> ~9.398 ns half-period.
module OSCH #(parameter NOM_FREQ = "53.2") (input STDBY, output reg OSC, output SEDSTDBY);
  initial OSC = 1'b0;
  always #9.398 if (!STDBY) OSC = ~OSC;   // free-running ~53.2 MHz (its own async domain vs clk12)
  assign SEDSTDBY = 1'b0;
endmodule
