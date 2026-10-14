module led(
    input                                 app_rx_data_valid       ,
    input                 [   7:   0]     app_rx_data             ,
    input  wire           [  15:   0]     app_rx_data_length      ,
    input                                 udp_rx_clk              ,
    input  wire                           reset                   ,
    input  wire                           mode_bp12_led_seg       ,  // 模式控制：0=LED模式，1=数码管模式

    output          [3:0] led_data_1 ,
    output reg      [7:0] dled ,
    output reg      [7:0] tub
);
reg  [15:0] scan_div;
reg  [63:0]led_data;
reg                [  15:   0]     cnt                         ;
reg                [   1:   0]     state                       ;


//// ---------------------- 数码管段选解码寄存器 ----------------------
//reg                [  7:   0]     seg_decode                 ;  // 解码后的7位段选信号

// 8位数码管控制寄存器
reg  [3:0]                               bcd_code[7:0];  // 8组4位BCD码，对应8位数码管
reg  [7:0]                               seg_decode[7:0];// 8组7段位选信号
reg  [2:0]                               scan_cnt;       // 数码管扫描计数器（0~7）


// ---------------------- 3. 原有计数器逻辑（保持不变） ----------------------
always @(posedge udp_rx_clk or negedge reset) begin
    if(!reset)  begin
        cnt   <=16'b0;
    end
    else if (app_rx_data_valid & cnt<(app_rx_data_length-1))begin
        cnt<=cnt+1;
    end
    else if (app_rx_data_valid & cnt==(app_rx_data_length-1))
        cnt<=16'b0;
    else
        cnt<=cnt;
end


// ---------------------- 4. 数据接收逻辑 ----------------------
// 支持三种数据包长度：
//   3字节: LED控制（兼容模式）
//   5字节: 数码管控制（数码管模式）
//   8字节: 数码管+单色LED控制（数码管模式完整包）
always @(posedge udp_rx_clk or negedge reset) begin
    if(!reset)
        led_data<=64'b0;
    else if (app_rx_data_valid)
        case (cnt)
            0:led_data[63:56]<=app_rx_data;  // byte0: 单色LED（3字节包）或单色LED（8字节包）
            1:led_data[55:48]<=app_rx_data;  // byte1: 绿色LED（3字节包）或数码管数据（5/8字节包）
            2:led_data[47:40]<=app_rx_data;  // byte2: 红色LED（3字节包）或数码管数据（5/8字节包）
            3:led_data[39:32]<=app_rx_data;  // byte3: 数码管数据（5/8字节包）
            4:led_data[31:24]<=app_rx_data;  // byte4: 数码管数据（5/8字节包）
            5:led_data[23:16]<=app_rx_data;  // byte5: 保留（8字节包）
            6:led_data[15:8] <=app_rx_data;  // byte6: 保留（8字节包）
            7:led_data[7:0]  <=app_rx_data;  // byte7: 保留（8字节包）
        endcase
    else
        led_data <=led_data;
end

integer i;
// BCD码拆解：从led_data[55:24]提取8组4位数据
always @(posedge udp_rx_clk or negedge reset) begin
    if(!reset) begin
        for(i=0;i<8;i=i+1) begin
            bcd_code[i] <= 4'b0000;
        end
    end 
    else begin
        bcd_code[7] <= led_data[55:52];  
        bcd_code[6] <= led_data[51:48];
        bcd_code[5] <= led_data[47:44];
        bcd_code[4] <= led_data[43:40];
        bcd_code[3] <= led_data[39:36];
        bcd_code[2] <= led_data[35:32];
        bcd_code[1] <= led_data[31:28];
        bcd_code[0] <= led_data[27:24];  
    end
end




always @(posedge udp_rx_clk or negedge reset) begin
    if(!reset) begin
        for(i=0;i<8;i=i+1) begin
            seg_decode[i] <= 8'b11111111;  // 复位全灭
        end
    end 
    else begin
        // 第1位数码管解码
        case(bcd_code[0])
            4'h0: seg_decode[0] <= 8'b1100_0000;
            4'h1: seg_decode[0] <= 8'b1111_1001;
            4'h2: seg_decode[0] <= 8'b1010_0100;
            4'h3: seg_decode[0] <= 8'b1011_0000;
            4'h4: seg_decode[0] <= 8'b1001_1001;
            4'h5: seg_decode[0] <= 8'b1001_0010;
            4'h6: seg_decode[0] <= 8'b1000_0010;
            4'h7: seg_decode[0] <= 8'b1111_1000;
            4'h8: seg_decode[0] <= 8'b1000_0000;
            4'h9: seg_decode[0] <= 8'b1001_0000;

            default: seg_decode[0] <= 8'b1111_1111;
        endcase
        // 第2位数码管解码
        case(bcd_code[1])
            4'h0: seg_decode[1] <= 8'b1100_0000;
            4'h1: seg_decode[1] <= 8'b1111_1001;
            4'h2: seg_decode[1] <= 8'b1010_0100;
            4'h3: seg_decode[1] <= 8'b1011_0000;
            4'h4: seg_decode[1] <= 8'b1001_1001;
            4'h5: seg_decode[1] <= 8'b1001_0010;
            4'h6: seg_decode[1] <= 8'b1000_0010;
            4'h7: seg_decode[1] <= 8'b1111_1000;
            4'h8: seg_decode[1] <= 8'b1000_0000;
            4'h9: seg_decode[1] <= 8'b1001_0000;

            default: seg_decode[1] <= 8'b1111_1111;
        endcase
        case(bcd_code[2])
            4'h0: seg_decode[2] <= 8'b1100_0000;
            4'h1: seg_decode[2] <= 8'b1111_1001;
            4'h2: seg_decode[2] <= 8'b1010_0100;
            4'h3: seg_decode[2] <= 8'b1011_0000;
            4'h4: seg_decode[2] <= 8'b1001_1001;
            4'h5: seg_decode[2] <= 8'b1001_0010;
            4'h6: seg_decode[2] <= 8'b1000_0010;
            4'h7: seg_decode[2] <= 8'b1111_1000;
            4'h8: seg_decode[2] <= 8'b1000_0000;
            4'h9: seg_decode[2] <= 8'b1001_0010;

            default: seg_decode[2] <= 8'b1111_1111;
        endcase
        case(bcd_code[3])
            4'h0: seg_decode[3] <= 8'b1100_0000;
            4'h1: seg_decode[3] <= 8'b1111_1001;
            4'h2: seg_decode[3] <= 8'b1010_0100;
            4'h3: seg_decode[3] <= 8'b1011_0000;
            4'h4: seg_decode[3] <= 8'b1001_1001;
            4'h5: seg_decode[3] <= 8'b1001_0010;
            4'h6: seg_decode[3] <= 8'b1000_0010;
            4'h7: seg_decode[3] <= 8'b1111_1000;
            4'h8: seg_decode[3] <= 8'b1000_0000;
            4'h9: seg_decode[3] <= 8'b1001_0000;

            default: seg_decode[3] <= 8'b1111_1111;
        endcase
        case(bcd_code[4])
            4'h0: seg_decode[4] <= 8'b1100_0000;
            4'h1: seg_decode[4] <= 8'b1111_1001;
            4'h2: seg_decode[4] <= 8'b1010_0100;
            4'h3: seg_decode[4] <= 8'b1011_0000;
            4'h4: seg_decode[4] <= 8'b1001_1001;
            4'h5: seg_decode[4] <= 8'b1001_0010;
            4'h6: seg_decode[4] <= 8'b1000_0010;
            4'h7: seg_decode[4] <= 8'b1111_1000;
            4'h8: seg_decode[4] <= 8'b1000_0000;
            4'h9: seg_decode[4] <= 8'b1001_0000;

            default: seg_decode[4] <= 8'b1111_1111;
        endcase
        case(bcd_code[5])
            4'h0: seg_decode[5] <= 8'b1100_0000;
            4'h1: seg_decode[5] <= 8'b1111_1001;
            4'h2: seg_decode[5] <= 8'b1010_0100;
            4'h3: seg_decode[5] <= 8'b1011_0000;
            4'h4: seg_decode[5] <= 8'b1001_1001;
            4'h5: seg_decode[5] <= 8'b1001_0010;
            4'h6: seg_decode[5] <= 8'b1000_0010;
            4'h7: seg_decode[5] <= 8'b1111_1000;
            4'h8: seg_decode[5] <= 8'b1000_0000;
            4'h9: seg_decode[5] <= 8'b1001_0000;

            default: seg_decode[5] <= 8'b1111_1111;
        endcase
        case(bcd_code[6])
            4'h0: seg_decode[6] <= 8'b1100_0000;
            4'h1: seg_decode[6] <= 8'b1111_1001;
            4'h2: seg_decode[6] <= 8'b1010_0100;
            4'h3: seg_decode[6] <= 8'b1011_0000;
            4'h4: seg_decode[6] <= 8'b1001_1001;
            4'h5: seg_decode[6] <= 8'b1001_0010;
            4'h6: seg_decode[6] <= 8'b1000_0010;
            4'h7: seg_decode[6] <= 8'b1111_1000;
            4'h8: seg_decode[6] <= 8'b1000_0000;
            4'h9: seg_decode[6] <= 8'b1001_0000;

            default: seg_decode[6] <= 8'b1111_1111;
        endcase
        case(bcd_code[7])
            4'h0: seg_decode[7] <= 8'b1100_0000;
            4'h1: seg_decode[7] <= 8'b1111_1001;
            4'h2: seg_decode[7] <= 8'b1010_0100;
            4'h3: seg_decode[7] <= 8'b1011_0000;
            4'h4: seg_decode[7] <= 8'b1001_1001;
            4'h5: seg_decode[7] <= 8'b1001_0010;
            4'h6: seg_decode[7] <= 8'b1000_0010;
            4'h7: seg_decode[7] <= 8'b1111_1000;
            4'h8: seg_decode[7] <= 8'b1000_0000;
            4'h9: seg_decode[7] <= 8'b1001_0000;

            default: seg_decode[7] <= 8'b1111_1111;
        endcase
    end
end



// 扫描计数器：0~7循环计数，频率约1kHz（根据udp_rx_clk调整）
always @(posedge udp_rx_clk or negedge reset) begin
    if(!reset) begin
        scan_div <= 16'd0;
        scan_cnt <= 3'b000;
    end
    else begin
        scan_div <= scan_div + 1'b1;
        // 检测scan_div溢出（从65535到0）
        if(scan_div == 16'hFFFF) begin
            scan_cnt <= scan_cnt + 1'b1;
        end
    end
end


// 段选总线与位选信号输出
// 修改：根据外部mode_bp12_led_seg信号判断模式，而不是led_data[59:56]
always @(posedge udp_rx_clk or negedge reset) begin
    if(!reset) begin
        tub  <= 8'b11111111;  // 复位时tub灭
        dled <= 8'b11111111;  // 复位时dled灭
    end
    else begin
        case(mode_bp12_led_seg)  // 根据外部模式控制信号判断输出模式
            1'b0: begin  // LED模式：输出双色LED数据
                tub  <= led_data[47:40];  // tub取led_data[47:40]（红色LED）
                dled <= led_data[55:48];  // dled取led_data[55:48]（绿色LED）
            end
            1'b1: begin  // 数码管模式：输出数码管扫描数据
                case(scan_cnt)  // 按扫描计数器切换8位数码管
                    3'd0: begin tub <= seg_decode[7]; dled <= 8'b11111110; end  // 第1位亮
                    3'd1: begin tub <= seg_decode[6]; dled <= 8'b11111101; end  // 第2位亮
                    3'd2: begin tub <= seg_decode[5]; dled <= 8'b11111011; end  // 第3位亮
                    3'd3: begin tub <= seg_decode[4]; dled <= 8'b11110111; end  // 第4位亮
                    3'd4: begin tub <= seg_decode[3]; dled <= 8'b11101111; end  // 第5位亮
                    3'd5: begin tub <= seg_decode[2]; dled <= 8'b11011111; end  // 第6位亮
                    3'd6: begin tub <= seg_decode[1]; dled <= 8'b10111111; end  // 第7位亮
                    3'd7: begin tub <= seg_decode[0]; dled <= 8'b01111111; end  // 第8位亮
                    default: begin tub <= 8'b11111111; dled <= 8'b11111111; end
                endcase
            end
            default: begin  // 其他值：保持灭灯状态
                tub  <= 8'b11111111;
                dled <= 8'b11111111;
            end
        endcase
    end
end







assign  led_data_1 = led_data[63:60];

endmodule