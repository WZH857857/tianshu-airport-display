`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2019/07/9 22:52:49
// Design Name: bounding box
// Module Name: position
// Project Name: bounding_box
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ns

module bounding_box(
		//system signals 
		input	        	rst_n		,
		input	        	sclk		,
		//image input 
		input				vsync_i		,
		input           	hsync_i		,
		input	        	idata		,
		input		[23:0]	rgb_data_i	, 
		//inmage output
		output				vsync_o		,
		output				hsync_o		,
		output	reg	[23:0]	rgb_data_o	,
		//coor output
		output  wire [10:0] coor_data
);

//========================================================================\
// =========== Define Parameter and Internal signals =========== 
//========================================================================/

reg	  	[10:0]   	hcout			; 
reg	  	[10:0]   	vcout			;
reg 	[23:0]		rgb_data_r0		;
reg 	[23:0]		rgb_data_r1		;
reg 	[23:0]		rgb_data_r2		;
reg 	[23:0]		rgb_data_r3		;
reg 	[23:0]		rgb_data_r4		;
reg 	[23:0]		rgb_data_r5		;

reg 				vsync_i_r		;
reg 				hsync_i_r		;
reg 				idata_r			;

//=============================================================================
//****************************     Main Code    *******************************
//=============================================================================

always @ (posedge sclk or negedge rst_n) begin
	if(rst_n == 1'b0)
		vsync_i_r <= 1'b0;
	else
		vsync_i_r <= vsync_i;	   
end

always @ (posedge sclk or negedge rst_n) begin
	if(rst_n == 1'b0)
		hsync_i_r <= 1'b0;
	else
		hsync_i_r <= hsync_i;	   
end
			
always @ (posedge sclk or negedge rst_n) begin
	if(rst_n == 1'b0)
		idata_r <= 1'b0;
	else
		idata_r <= idata;	   
end


always @ (posedge sclk or negedge rst_n) begin
	if(rst_n == 1'b0) begin
		rgb_data_r0	<= 24'd0;
		rgb_data_r1	<= 24'd0;
		rgb_data_r2	<= 24'd0;
		rgb_data_r3	<= 24'd0;
		rgb_data_r4	<= 24'd0;
		rgb_data_r5	<= 24'd0;
	end	
	else begin
		rgb_data_r0	<= rgb_data_i;
		rgb_data_r1	<= rgb_data_r0;
		rgb_data_r2	<= rgb_data_r1;
		rgb_data_r3	<= rgb_data_r2;
		rgb_data_r4	<= rgb_data_r3;
		rgb_data_r5	<= rgb_data_r4;
	end	
        
end
			
//h_count
always@(posedge sclk or negedge rst_n)
	if(!rst_n)
		hcout <= 11'd0;
	else if(hsync_i==1)
		hcout <= hcout +1'b1;
	else
		hcout <= 11'd0;

//v_count
always@(posedge sclk or negedge rst_n)
	if(!rst_n)
		vcout <= 11'd0;
	else if(~vsync_i_r & vsync_i)
		vcout <= 11'd0;
	else if(~hsync_i & hsync_i_r)
		vcout <= vcout  + 1;
	else
		vcout <= vcout;

reg [10:0] x_maxr; 

always @( posedge sclk  or  negedge rst_n)
	if (!rst_n) 
		x_maxr  <=  0;
	else if(~vsync_i_r & vsync_i) 
		x_maxr  <=  0;
	else if(hsync_i_r==1) 
		begin
			if ((idata_r==1'b1)&&(x_maxr <= hcout))
				x_maxr  <=  hcout;
			else
				x_maxr<=x_maxr;
		end		
		
	
reg [10:0] x_minr;

always @( posedge sclk or negedge rst_n)
	if (!rst_n)
		x_minr  <=11'd640;
	else  if( ~vsync_i_r & vsync_i )
		x_minr  <=  11'd640;
	else  if(hsync_i_r==1) 
		begin
			if ((idata_r==1)&&(x_minr>hcout ))
				x_minr  <=  hcout;
			else
				x_minr <=x_minr;
		end

reg [10:0] y_maxr;
always @( posedge sclk  or  negedge rst_n)
	if (!rst_n)
		y_maxr  <=   0;
	else  if(~vsync_i_r & vsync_i)
		y_maxr  <=   0;
	else  if(hsync_i_r)
		begin
			if ((idata_r==1)&&(y_maxr <= vcout ))
				y_maxr  <=   vcout;
			else
				y_maxr <=y_maxr;
		end

reg [10:0]y_minr;

always @( posedge sclk  or  negedge rst_n) 
	if (!rst_n)
		y_minr  <=   11'd480;
	else  if(~vsync_i_r & vsync_i)
		y_minr  <=   11'd480;
	else  if(hsync_i_r)
		begin
			if ((idata_r==1)&&(y_minr>vcout))
				y_minr  <=   vcout;
			else
				y_minr<=y_minr;
		end
	
reg [10:0]x_max,y_max,x_min,y_min; 

always @( posedge sclk  or  negedge rst_n)
	if (!rst_n) 
		begin
			x_max  <=   0;
			y_max  <=   0;
			x_min  <=   0;
			y_min  <=   0;
		end
	else  if(~vsync_i_r & vsync_i)
		begin
			x_max  <=   x_maxr;
			y_max  <=   y_maxr;
			x_min  <=   x_minr;
			y_min  <=   y_minr;
		end		
	
always@(posedge sclk or negedge rst_n)//琛屾壂鎻?
		if(!rst_n)
			rgb_data_o<='d0;
		else if(hsync_i_r)
			begin
				if((((hcout==x_min)|(hcout==x_min+1)|(hcout==x_max)|(hcout==x_max+1))&&((vcout>=y_min)&&(vcout<=y_max))) ||
				   (((vcout==y_min)|(vcout==y_min+1)|(vcout==y_max)|(vcout==y_max+1))&&((hcout>=x_min)&&(hcout<=x_max))))
					rgb_data_o<=24'h0000ff;
				else
					rgb_data_o<=rgb_data_r5;
			end
		else
			rgb_data_o<=0;	

assign  vsync_o	 = vsync_i_r;
assign  hsync_o	 = hsync_i_r;
assign  coor_data = x_max[10:1] + x_min[10:1];
endmodule 
