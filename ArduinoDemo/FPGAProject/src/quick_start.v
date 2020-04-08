`timescale 1ns / 10ps 
module quick_start(
                   fpga_clk_in,
                   fpga_rst_n ,
                   hw_led     ,  
                   io_led1    ,   
                   io_led2    ,    
                   button1	  ,    
                   button2    ,
                   pwmout9	  , 
                   exti_out
                   );
                   
//---------------------------------------                                         
//Define input / output                                                           
//---------------------------------------                                         
input        fpga_clk_in;//global clock input
input        fpga_rst_n ;
output       hw_led     ;//fpga fabric drive led
inout        io_led1    ;//MCU drive led
inout        io_led2    ;//MCU drive led
inout        button1    ;
inout        button2    ;
output		 pwmout9	;/* synthesis keep */
output		 exti_out	;

//---------------------------------------                                         
//sys_pll module                                                         
//--------------------------------------- 
wire clk25	;
wire clk200	;
sys_pll U_SYS_PLL(
                  .refclk  (fpga_clk_in),
                  .extlock (           ),
                  .clk0_out(clk200     ),
                  .clk1_out(clk25      )
                  );
                   
//---------------------------------------                                         
//MCU module                                                         
//---------------------------------------                   
wire gpio_h0_out;
wire gpio_h0_en ;          
wire gpio_h1_out;               
wire gpio_h1_en ; 
wire gpio_h2_out;                 
wire gpio_h2_en ;  
wire gpio_h3_out;               
wire gpio_h3_en ; 
// if gpio_h0_en = 1'b0, gpio_h0 is output; 
// if gpio_h0_en = 1'b1, gpio_h0 is input ;
// the enable configured by Software  
assign io_led1 = gpio_h0_en ? 1'bz : gpio_h0_out; 
assign button1 = gpio_h1_en ? 1'bz : gpio_h1_out;    
assign button2 = gpio_h2_en ? 1'bz : gpio_h2_out; 
assign io_led2 = gpio_h3_en ? 1'bz : gpio_h3_out;  
AL_MCU U_AL_MCU(
                .ppm_clk     (clk25      ),//MCU clk input  
                
                .gpio_h0_out (gpio_h0_out),//gp_h0 output port   
                .gpio_h0_oe_n(gpio_h0_en ),//tri enable          
                .gpio_h0_in  (io_led1    ),//gp_h0 input port    
                         
                .gpio_h1_out (gpio_h1_out),//gp_h1 output port          
                .gpio_h1_oe_n(gpio_h1_en ),//tri enable       
                .gpio_h1_in  (button1    ),//gp_h1 input port    
                  
                .gpio_h2_out (gpio_h2_out),//gp_h1 output port    
                .gpio_h2_oe_n(gpio_h2_en ),//tri enable     
                .gpio_h2_in  (button2    ), //gp_h1 input port  
                      
                .gpio_h3_out (gpio_h3_out),//gp_h3 output port  
                .gpio_h3_oe_n(gpio_h3_en ),//tri enable   
                .gpio_h3_in  (io_led2    )//gp_h3 input port   
                );
 
//---------------------------------------                                        
//bram256kbit module                             
//---------------------------------------  
wire 	[31:0]	bram32k_out ;	/* synthesis keep */
reg 	[12:0]	bram32k_addr;	/* synthesis keep */ 
reg 	[7:0]	pwm_compare9;
parameter PWM_ADDR0 = 13'd8100;
parameter PWM_ADDR1 = 13'd8101;
parameter PWM_ADDR2 = 13'd8102; 
parameter PWM_ADDR3 = 13'd8103;
parameter PWM_ADDR4 = 13'd8104; 
parameter PWM_ADDR5 = 13'd8105; 
parameter PWM_ADDR6 = 13'd8106;
parameter PWM_ADDR7 = 13'd8107;
parameter PWM_ADDR8 = 13'd8108;
parameter PWM_ADDR9 = 13'd8109;
always @(posedge clk25 or negedge fpga_rst_n)
begin
    if(!fpga_rst_n)begin
        bram32k_addr <= 13'd10;

    end
    else begin
    	bram32k_addr <= bram32k_addr + 1'd1;
    	case(bram32k_addr)
        	PWM_ADDR0 + 13'd2	:;
        	PWM_ADDR1 + 13'd2	:;
        	PWM_ADDR2 + 13'd2	:;
        	PWM_ADDR3 + 13'd2	:;
        	PWM_ADDR4 + 13'd2	:;
        	PWM_ADDR5 + 13'd2	:;
        	PWM_ADDR6 + 13'd2	:;
        	PWM_ADDR7 + 13'd2	:;
        	PWM_ADDR8 + 13'd2	:;
        	PWM_ADDR9 + 13'd2	:pwm_compare9 <= bram32k_out[7:0];
        	default:;
		endcase
    end

end
bram256kbit u_bram256kbit( 
                         .dob    (bram32k_out ),
                         .dib    (            ),
                         .ceb    (1'b1        ),
                         .oceb   (1'b1        ),   
                         .clkb   (clk25       ),   
                         .web    (1'b0        ),
                         .rstb   (1'b0        ),
                         .addrb  (bram32k_addr),
                         .webbyte(4'b0000     ) 
                         ); 

//---------------------------------------                                        
//32bits counter                             
//---------------------------------------                 
reg [31:0] led_cnt;
always @(posedge clk25 or negedge fpga_rst_n)
begin
    if(~fpga_rst_n)
        led_cnt <= 32'd0;  
    else
    	led_cnt <= led_cnt + 1'b1 ; 
end
assign hw_led = {led_cnt[24]};	//use the 24 bit to drive led
assign exti_out = {led_cnt[24]};

//---------------------------------------                                         
//PWM Generator                                                                           
//---------------------------------------                
reg	[9:0] pwm;          
always @(posedge clk25 or negedge fpga_rst_n) 
begin      
    if(~fpga_rst_n)            
        pwm[9] <= 1'b1;             
    else begin 
    	pwm[9] <= (led_cnt[7:0] < pwm_compare9) ? 1'b0 : 1'b1;  
    end
end
assign pwmout9 = pwm[9];

endmodule 