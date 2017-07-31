

`timescale 1ns / 1ns // `timescale time_unit/time_precision

//SW[11:0] input for data
//SW[13] input for clock
//SW[17:15] input to select the function

//LEDR[17:0] output LED display for binary representation of ALUout

module final_project(LEDR, CLOCK_50, VGA_CLK, VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_R, VGA_B, VGA_G, KEY, SW);
    input [17:0] SW; //input switches
    input CLOCK_50;
	 input [1:0] KEY;
    output [17:0] LEDR; //red output LEDs

	 // Declare your inputs and outputs here
    // Do not change the following outputs
    output          VGA_CLK;                //  VGA Clock
    output          VGA_HS;                 //  VGA H_SYNC
    output          VGA_VS;                 //  VGA V_SYNC
    output          VGA_BLANK_N;                //  VGA BLANK
    output          VGA_SYNC_N;             //  VGA SYNC
    output  [9:0]   VGA_R;                  //  VGA Red[9:0]
    output  [9:0]   VGA_G;                  //  VGA Green[9:0]
    output  [9:0]   VGA_B;                  //  VGA Blue[9:0]

    // Create an Instance of a VGA controller - there can be only one!
    // Define the number of colours as well as the initial background
    // image file (.MIF) for the controller.
    vga_adapter VGA(
            .resetn(1'b1),
            .clock(CLOCK_50),
            .colour(colour),
            .x(x),
            .y(y),
            .plot(writeEn),
            /* Signals for the DAC to drive the monitor. */
            .VGA_R(VGA_R),
            .VGA_G(VGA_G),
            .VGA_B(VGA_B),
            .VGA_HS(VGA_HS),
            .VGA_VS(VGA_VS),
            .VGA_BLANK(VGA_BLANK_N),
            .VGA_SYNC(VGA_SYNC_N),
            .VGA_CLK(VGA_CLK));
        defparam VGA.RESOLUTION = "160x120";
        defparam VGA.MONOCHROME = "FALSE";
        defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
        defparam VGA.BACKGROUND_IMAGE = "background.mif";
	 
    main master(
        .clock_50(CLOCK_50),
        .data(SW[11:0]),
        .clock(KEY[0]),
        .LEDoutput(LEDR)
        );

endmodule

module main(clock_50, data, clock, LEDoutput);
    input [11:0] data;  // value of 12-bit user input number
    input clock_50;
    input clock;  // clock signal used for the register
    output [17:0] LEDoutput;
	 
	 wire [33:0] results;

    assign results[33:16] = LEDoutput[17:0];

    function_one func1(
        .a(clock_50),
        .result(results[0])
        );

    function_two func2(
        .clock(results[3]),
		.load_n(data[0]),
        .result(LEDoutput[17:0])
        );
		  
    function_2 func22(
        .a(clock_50),
        .result(results[1])
        );


    function_3 func33333(
        .a(clock_50),
        .result(results[2])
        );

		  
	 mux3 muxxx(
		.select(data[11:10]),
		.d(results[2:0]),
		.q(results[3])
		);

endmodule

module function_one(a, result);
    input a; //clock_50
	 wire enable;
	 
	 reg flip;
    output reg [7:0] result; 

    RateDivider RD1(
        .clock(a),
        .enable(enable),
        );
		  

    always @ (posedge enable)

    begin
        if (flip == 1'b0)
		  begin
				result = 1'b1; //if the select signal is 0, set the result as input 0
				flip = 1'b1;
		  end
		  else if (flip == 1'b1)
		  begin
				result = 1'b0;
				flip = 1'b0;
		  end
    end

endmodule

module function_2(a, result);
    input a; //clock_50
    wire enable;
	 
	 reg flip;
    output reg result; 

	 RateDividerTwo RD2(
        .clock(a),
        .enable(enable),
        );

    always @ (posedge enable)

    begin
        if (flip == 1'b0)
		  begin
				result = 1'b1; //if the select signal is 0, set the result as input 0
				flip = 1'b1;
		  end
		  else if (flip == 1'b1)
		  begin
				result = 1'b0;
				flip = 1'b0;
		  end
    end

endmodule

module function_3(a, result);
    input a; //clock_50
    wire enable;
	 
	 reg flip;
    output reg result; 
	
	 RateDividerThree RD3(
        .clock(a),
        .enable(enable),
        );

    always @ (posedge enable)
	 
    begin
        if (flip == 1'b0)
		  begin
				result = 1'b1; //if the select signal is 0, set the result as input 0
				flip = 1'b1;
		  end
		  else if (flip == 1'b1)
		  begin
				result = 1'b0;
				flip = 1'b0;
		  end
    end

endmodule

module mux3(select, d, q );

	input[1:0] select;
	input[3:0] d;
	output q;

	reg q;
	wire[1:0] select;
	wire[3:0] d;

	always @( select or d )
	begin
		if( select == 0)
			q = d[0];

		if( select == 1)
			q = d[1];

		if( select == 2)
			q = d[2];

		if( select == 3)
			q = d[3];
	end

endmodule



module function_two(clock, load_n, result);
    input clock;
	 input load_n;

    wire [319:0] shifterbit;
    wire timetopressleft;

    output reg [17:0] result;

    register320bit displayleft(
        .inputnum(319'b00000000000000000000000000000000000000000000000000000000000011100000000000000000000000000000000000111000000011100000111000000001110000000000000000001110000000000000000000000000000000000000111000000000000000001110000000000000000000000000000000000000111000000000000000001110000000000000000000000000000000000000000001111110),
        .clock(clock),
        .clear_n(0),
		  .load_n(load_n),
        .q(shifterbit),
        .outputbutton(timetopressleft)
        );

    always @ (negedge clock)
    begin
        result[17:0] = shifterbit[17:0];
    end
endmodule

module RateDivider(clock, enable);
    // Clock input should be CLOCK_50
    input clock;

    // Uncomment this next line if you're implementing other
    // BPMs.
    // input [1:0] func;

    // Every posedge of enable means a 10th of a beat has passed.
    output reg enable;

    reg [24:0] q = 25'b0000000000000000000000000;
    // This time is the number of clock cycles it takes to
    // go through 1/20th of a beat. So Every 20 of these cycles
    // Makes up a whole beat. Every posedge of the output
    // is 1/10th of the beat.
    reg [24:0] d = 25'b0111001001110000111000000;
    always @(posedge clock)
    begin
        if (q == d)
		  begin
            q <= 0;
            enable = 1'b1;
			end
        else
		  begin
            q <= q + 1'b1;
            enable = 1'b0;
			end
    end
endmodule

module RateDividerTwo(clock, enable);
    // Clock input should be CLOCK_50
    input clock;

    // Uncomment this next line if you're implementing other
    // BPMs.
    // input [1:0] func;

    // Every posedge of enable means a 10th of a beat has passed.
    output reg enable;

    reg [24:0] q = 25'b0000000000000000000000000;
    // This time is the number of clock cycles it takes to
    // go through 1/20th of a beat. So Every 20 of these cycles
    // Makes up a whole beat. Every posedge of the output
    // is 1/10th of the beat.
    reg [24:0] d = 25'b0011001001110000111000000;
    always @(posedge clock)
    begin
        if (q == d)
		  begin
            q <= 0;
            enable = 1'b1;
			end
        else
		  begin
            q <= q + 1'b1;
            enable = 1'b0;
			end
    end
endmodule

module RateDividerThree(clock, enable);
    // Clock input should be CLOCK_50
    input clock;

    // Uncomment this next line if you're implementing other
    // BPMs.
    // input [1:0] func;

    // Every posedge of enable means a 10th of a beat has passed.
    output reg enable;

    reg [24:0] q = 25'b0000000000000000000000000;
    // This time is the number of clock cycles it takes to
    // go through 1/20th of a beat. So Every 20 of these cycles
    // Makes up a whole beat. Every posedge of the output
    // is 1/10th of the beat.
    reg [24:0] d = 25'b0001001001110000111000000;
    always @(posedge clock)
    begin
        if (q == d)
		  begin
            q <= 0;
            enable = 1'b1;
			end
        else
		  begin
            q <= q + 1'b1;
            enable = 1'b0;
			end
    end
endmodule

module register320bit(inputnum, clock, load_n, clear_n, q, outputbutton);
    input clock; // bit used to trigger the registe
	 input clear_n;
    input load_n; // reset bit for the register
    input [319:0] inputnum;
	 
	 reg [319:0] shifterbit;

    output reg outputbutton = 1'b0;

    output reg [119:0] q;
    // data currently stored in the register

    always @ (negedge clock) // triggered every time clock falls
    begin
        if (clear_n == 1)  // when clear_n is 1
            q <= 0;  // q is set to 0
		  else if (load_n == 1)
				shifterbit = inputnum;
        else  // when reset_n is not 1
		  begin
            shifterbit[318:0] = shifterbit[319:1];
            shifterbit[319] <= 0;
            q[119:0] = shifterbit[119:0];
            // The 10th last value should be where the line to hit the thing is at.
            outputbutton <= q[10]; // THIS OUTPUTS THE 11TH LAST VALUE. We'll use the 11th last value for timing the button presses. I'll handle this
            // don't worry about it.
			end
    end
endmodule
