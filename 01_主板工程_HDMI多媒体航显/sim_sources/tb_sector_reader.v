`timescale 1ns / 1ps
//=============================================================================
// tb_sector_reader.v - unit test for the 512 byte sector buffer
//=============================================================================
module tb_sector_reader;

    reg         clk;
    reg         rst_n;
    reg         start;
    reg  [31:0] lba;
    wire        busy;
    wire        ready;
    wire        error;
    wire [3:0]  error_code;
    reg  [8:0]  raddr0;
    wire [7:0]  rdata0;
    reg         release_req;

    reg         sd_init_done;
    wire        sd_sec_read;
    wire [31:0] sd_sec_read_addr;
    reg  [7:0]  sd_sec_read_data;
    reg         sd_sec_read_data_valid;
    reg         sd_sec_read_end;
    reg         sd_sec_read_error;
    reg  [3:0]  sd_sec_read_error_code;

    integer i;
    integer errors;

    sector_reader #(
        .TIMEOUT_CYCLES(24'd2000)
    ) dut (
        .clk                   (clk),
        .rst_n                 (rst_n),
        .start                 (start),
        .lba                   (lba),
        .busy                  (busy),
        .ready                 (ready),
        .error                 (error),
        .error_code            (error_code),
        .raddr0                (raddr0),
        .raddr1                (9'd0),
        .raddr2                (9'd0),
        .raddr3                (9'd0),
        .rdata0                (rdata0),
        .rdata1                (),
        .rdata2                (),
        .rdata3                (),
        .release_req           (release_req),
        .sd_init_done          (sd_init_done),
        .sd_sec_read           (sd_sec_read),
        .sd_sec_read_addr      (sd_sec_read_addr),
        .sd_sec_read_data      (sd_sec_read_data),
        .sd_sec_read_data_valid(sd_sec_read_data_valid),
        .sd_sec_read_end       (sd_sec_read_end),
        .sd_sec_read_error     (sd_sec_read_error),
        .sd_sec_read_error_code(sd_sec_read_error_code)
    );

    always #10 clk = ~clk;      // 50 MHz

    task send_sector_byte;
        input [7:0] value;
        begin
            @(negedge clk);
            sd_sec_read_data       = value;
            sd_sec_read_data_valid = 1'b1;
            @(posedge clk);
            @(negedge clk);
            sd_sec_read_data_valid = 1'b0;
        end
    endtask

    task release_sector;
        begin
            @(negedge clk);
            release_req = 1'b1;
            @(posedge clk);
            @(negedge clk);
            release_req = 1'b0;
            @(posedge clk);
        end
    endtask

    initial begin
        clk                    = 1'b0;
        rst_n                  = 1'b0;
        start                  = 1'b0;
        lba                    = 32'h1234;
        raddr0                 = 9'd0;
        release_req            = 1'b0;
        sd_init_done           = 1'b1;
        sd_sec_read_data       = 8'd0;
        sd_sec_read_data_valid = 1'b0;
        sd_sec_read_end        = 1'b0;
        sd_sec_read_error      = 1'b0;
        errors                 = 0;

        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        //------------------------------------------------------------------
        // normal sector
        //------------------------------------------------------------------
        @(negedge clk);
        start = 1'b1;
        @(posedge clk);
        @(negedge clk);
        start = 1'b0;

        for (i = 0; i < 512; i = i + 1)
            send_sector_byte(i[7:0] ^ 8'hA5);

        @(negedge clk);
        sd_sec_read_end = 1'b1;
        @(posedge clk);
        @(negedge clk);
        sd_sec_read_end = 1'b0;

        wait (ready == 1'b1);
        if (busy !== 1'b0) begin
            $display("FAIL: busy stayed high after sector end");
            errors = errors + 1;
        end

        for (i = 0; i < 512; i = i + 1) begin
            raddr0 = i[8:0];
            #1;
            if (rdata0 !== (i[7:0] ^ 8'hA5)) begin
                $display("FAIL: sector byte %0d got %02x expected %02x",
                         i, rdata0, i[7:0] ^ 8'hA5);
                errors = errors + 1;
                i = 512;
            end
        end

        release_sector();
        if (ready !== 1'b0) begin
            $display("FAIL: ready did not clear on release");
            errors = errors + 1;
        end

        //------------------------------------------------------------------
        // short sector must report an error
        //------------------------------------------------------------------
        @(negedge clk);
        start = 1'b1;
        @(posedge clk);
        @(negedge clk);
        start = 1'b0;
        for (i = 0; i < 8; i = i + 1)
            send_sector_byte(i[7:0]);
        @(negedge clk);
        sd_sec_read_end = 1'b1;
        @(posedge clk);
        @(negedge clk);
        sd_sec_read_end = 1'b0;
        wait (error == 1'b1);
        release_sector();

        //------------------------------------------------------------------
        // timeout path
        //------------------------------------------------------------------
        @(negedge clk);
        start = 1'b1;
        @(posedge clk);
        @(negedge clk);
        start = 1'b0;
        wait (error == 1'b1);
        if (sd_sec_read !== 1'b0) begin
            $display("FAIL: sd_sec_read stayed asserted after timeout");
            errors = errors + 1;
        end
        release_sector();

        if (errors == 0)
            $display("RESULT: PASS tb_sector_reader");
        else
            $display("RESULT: FAIL tb_sector_reader errors=%0d", errors);

        $finish;
    end

    initial begin
        #2_000_000;
        $display("RESULT: FAIL tb_sector_reader timeout");
        $finish;
    end

endmodule
