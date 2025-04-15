
module d_ff
(
    input Clk,reset,enable,
    input [4:0] D,
    output [4:0] Q
);      

    wire C,e1;
    wire nClk;
    not (nClk,Clk);
    or(e1,enable,reset);
    and(C,nClk,e1);
    wire nreset;

    not (nreset,reset);
    wire Q0,D0;
    
    and (D0,nreset,D[0]);
    dff dff0 (.Clk(C | reset),.D(D0),.Q(Q[0]));
    
    wire D1;
    and (D1,nreset,D[1]);
    dff dff1 (.Clk(C | reset),.D(D1),.Q(Q[1]));

    wire D2;
    and (D2,nreset,D[2]);
    dff dff2 (.Clk(C | reset),.D(D2),.Q(Q[2]));
    
    wire D3;
    and (D3,nreset,D[3]);
    dff dff3 (.Clk(C | reset),.D(D3),.Q(Q[3]));
    
    wire D4;
    and (D4,nreset,D[4]);
    dff dff4 (.Clk(C | reset),.D(D4),.Q(Q[4]));
    
endmodule


module dff(   
    input Clk,D,
    output Q
);
    wire S,R,nD,nQ;
    not (nD,D);

    and (R,Clk,nD);
    and (S,Clk,D);
  
    nor (Q,R,nQ);
    nor (nQ,S,Q);

endmodule


module decoder (
    input [3:0] address,
    output [3:0] row_enable,
    output [3:0] col_enable
);

    wire n3, n2, n1, n0;

    not (n3,address[3]);
    not (n2,address[2]);
    not (n1,address[1]);
    not (n0,address[0]);
     
    // one hot encoding use karke  row and coloum enable lines banaya
    and (row_enable[0],n3,n2); 
    and (row_enable[1],n3,address[2]);
    and (row_enable[2],address[3],n2);
    and (row_enable[3],address[3],address[2]); 

 
    and (col_enable[0],n1,n0);
    and (col_enable[1],n1,address[0]);
    and (col_enable[2],address[1],n0); 
    and (col_enable[3],address[1],address[0]); 

endmodule

module mux_16to1(
    input  [4:0] in0,in1,in2,in3,in4,in5,in6,in7,in8,in9,in10,in11,in12,in13,in14,in15, 
    input  [3:0] sel,  
    output [4:0] out   
);
    wire sel0_n, sel1_n, sel2_n, sel3_n;
    wire s0, s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, s12, s13, s14, s15;

    not (sel0_n, sel[0]);
    not (sel1_n, sel[1]);
    not (sel2_n, sel[2]);
    not (sel3_n, sel[3]);

    and (s0, sel3_n, sel2_n, sel1_n, sel0_n);
    and (s1, sel3_n, sel2_n, sel1_n, sel[0]);
    and (s2, sel3_n, sel2_n, sel[1], sel0_n);
    and (s3, sel3_n, sel2_n, sel[1], sel[0]);
    and (s4, sel3_n, sel[2], sel1_n, sel0_n);
    and (s5, sel3_n, sel[2], sel1_n, sel[0]);
    and (s6, sel3_n, sel[2], sel[1], sel0_n);
    and (s7, sel3_n, sel[2], sel[1], sel[0]);
    and (s8, sel[3], sel2_n, sel1_n, sel0_n);
    and (s9, sel[3], sel2_n, sel1_n, sel[0]);
    and (s10, sel[3], sel2_n, sel[1], sel0_n);
    and (s11, sel[3], sel2_n, sel[1], sel[0]);
    and (s12, sel[3], sel[2], sel1_n, sel0_n);
    and (s13, sel[3], sel[2], sel1_n, sel[0]);
    and (s14, sel[3], sel[2], sel[1], sel0_n);
    and (s15, sel[3], sel[2], sel[1], sel[0]);

    and (out0_0,in0[0],s0), (out1_0,in1[0],s1), (out2_0,in2[0],s2), (out3_0,in3[0],s3);
    and (out4_0,in4[0],s4), (out5_0,in5[0],s5), (out6_0,in6[0],s6), (out7_0,in7[0],s7);
    and (out8_0,in8[0],s8), (out9_0,in9[0],s9), (out10_0,in10[0],s10), (out11_0,in11[0],s11);
    and (out12_0,in12[0],s12), (out13_0,in13[0],s13), (out14_0,in14[0],s14), (out15_0,in15[0],s15);
    
    or (out[0], out0_0, out1_0, out2_0, out3_0, out4_0, out5_0, out6_0, out7_0,
                out8_0, out9_0, out10_0, out11_0, out12_0, out13_0, out14_0, out15_0);

    and (out0_1, in0[1], s0), (out1_1, in1[1], s1), (out2_1, in2[1], s2), (out3_1, in3[1], s3);
    and (out4_1, in4[1], s4), (out5_1, in5[1], s5), (out6_1, in6[1], s6), (out7_1, in7[1], s7);
    and (out8_1, in8[1], s8), (out9_1, in9[1], s9), (out10_1, in10[1], s10), (out11_1, in11[1], s11);
    and (out12_1, in12[1], s12), (out13_1, in13[1], s13), (out14_1, in14[1], s14), (out15_1, in15[1], s15);

    or (out[1], out0_1, out1_1, out2_1, out3_1, out4_1, out5_1, out6_1, out7_1,
                out8_1, out9_1, out10_1, out11_1, out12_1, out13_1, out14_1, out15_1);

    and (out0_2, in0[2], s0), (out1_2, in1[2], s1), (out2_2, in2[2], s2), (out3_2, in3[2], s3);
    and (out4_2, in4[2], s4), (out5_2, in5[2], s5), (out6_2, in6[2], s6), (out7_2, in7[2], s7);
    and (out8_2, in8[2], s8), (out9_2, in9[2], s9), (out10_2, in10[2], s10), (out11_2, in11[2], s11);
    and (out12_2, in12[2], s12), (out13_2, in13[2], s13), (out14_2, in14[2], s14), (out15_2, in15[2], s15);

    or (out[2], out0_2, out1_2, out2_2, out3_2, out4_2, out5_2, out6_2, out7_2,
                out8_2, out9_2, out10_2, out11_2, out12_2, out13_2, out14_2, out15_2);

    and (out0_3, in0[3], s0), (out1_3, in1[3], s1), (out2_3, in2[3], s2), (out3_3, in3[3], s3);
    and (out4_3, in4[3], s4), (out5_3, in5[3], s5), (out6_3, in6[3], s6), (out7_3, in7[3], s7);
    and (out8_3, in8[3], s8), (out9_3, in9[3], s9), (out10_3, in10[3], s10), (out11_3, in11[3], s11);
    and (out12_3, in12[3], s12), (out13_3, in13[3], s13), (out14_3, in14[3], s14), (out15_3, in15[3], s15);

    or (out[3], out0_3, out1_3, out2_3, out3_3, out4_3, out5_3, out6_3, out7_3,
                out8_3, out9_3, out10_3, out11_3, out12_3, out13_3, out14_3, out15_3);

    and (out0_4, in0[4], s0), (out1_4, in1[4], s1), (out2_4, in2[4], s2), (out3_4, in3[4], s3);
    and (out4_4, in4[4], s4), (out5_4, in5[4], s5), (out6_4, in6[4], s6), (out7_4, in7[4], s7);
    and (out8_4, in8[4], s8), (out9_4, in9[4], s9), (out10_4, in10[4], s10), (out11_4, in11[4], s11);
    and (out12_4, in12[4], s12), (out13_4, in13[4], s13), (out14_4, in14[4], s14), (out15_4, in15[4], s15);

    or (out[4], out0_4, out1_4, out2_4, out3_4, out4_4, out5_4, out6_4, out7_4,
                out8_4, out9_4, out10_4, out11_4, out12_4, out13_4, out14_4, out15_4);

endmodule

// module d_ff(
//     input Clk,reset,enable,
//     input [4:0] D,
//     output reg [4:0] Q
// );
//     always @(posedge Clk or posedge reset) 
//     begin
//         if(reset)
//         begin
//             Q <= 5'b00000;
//         end  
//         else if(enable)
//         begin
//             Q <= D; 
//         end
//     end
// endmodule



module memory(  
    input Clk, write_enable, reset,
    input [4:0] data_in,
    input [3:0] address,
    output [4:0] data_out                  
);
   
    wire [3:0] row, col;
    wire [15:0] write;
    wire [4:0] q [15:0]; 

    decoder d(.address(address), .row_enable(row), .col_enable(col));

    d_ff m0 (.Clk(Clk), .reset(reset), .enable(row[0] & col[0] & write_enable), .D(data_in), .Q(q[0]));
    d_ff m1 (.Clk(Clk), .reset(reset), .enable(row[0] & col[1] & write_enable), .D(data_in), .Q(q[1]));
    d_ff m2 (.Clk(Clk), .reset(reset), .enable(row[0] & col[2] & write_enable), .D(data_in), .Q(q[2]));
    d_ff m3 (.Clk(Clk), .reset(reset), .enable(row[0] & col[3] & write_enable), .D(data_in), .Q(q[3]));
    d_ff m4 (.Clk(Clk), .reset(reset), .enable(row[1] & col[0] & write_enable), .D(data_in), .Q(q[4]));
    d_ff m5 (.Clk(Clk), .reset(reset), .enable(row[1] & col[1] & write_enable), .D(data_in), .Q(q[5]));
    d_ff m6 (.Clk(Clk), .reset(reset), .enable(row[1] & col[2] & write_enable), .D(data_in), .Q(q[6]));
    d_ff m7 (.Clk(Clk), .reset(reset), .enable(row[1] & col[3] & write_enable), .D(data_in), .Q(q[7]));
    d_ff m8 (.Clk(Clk), .reset(reset), .enable(row[2] & col[0] & write_enable), .D(data_in), .Q(q[8]));
    d_ff m9 (.Clk(Clk), .reset(reset), .enable(row[2] & col[1] & write_enable), .D(data_in), .Q(q[9]));
    d_ff m10 (.Clk(Clk), .reset(reset), .enable(row[2] & col[2] & write_enable), .D(data_in), .Q(q[10]));
    d_ff m11 (.Clk(Clk), .reset(reset), .enable(row[2] & col[3] & write_enable), .D(data_in), .Q(q[11]));
    d_ff m12 (.Clk(Clk), .reset(reset), .enable(row[3] & col[0] & write_enable), .D(data_in), .Q(q[12]));
    d_ff m13 (.Clk(Clk), .reset(reset), .enable(row[3] & col[1] & write_enable), .D(data_in), .Q(q[13]));
    d_ff m14 (.Clk(Clk), .reset(reset), .enable(row[3] & col[2] & write_enable), .D(data_in), .Q(q[14]));
    d_ff m15 (.Clk(Clk), .reset(reset), .enable(row[3] & col[3] & write_enable), .D(data_in), .Q(q[15]));

    mux_16to1 mux(
        .in0(q[0]), .in1(q[1]), .in2(q[2]), .in3(q[3]),
        .in4(q[4]), .in5(q[5]), .in6(q[6]), .in7(q[7]),
        .in8(q[8]), .in9(q[9]), .in10(q[10]), .in11(q[11]),
        .in12(q[12]), .in13(q[13]), .in14(q[14]), .in15(q[15]),
        .sel(address), 
        .out(data_out)
    );
endmodule


