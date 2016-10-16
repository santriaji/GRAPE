library ieee ;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_arith.all;


library ieee_proposed;
use ieee_proposed.fixed_pkg.all;

entity control is
generic ( left_size : integer := 9 );
port( 
        clk : in std_logic;
        reset : in std_logic;
		  dram_stall : in std_logic_vector(12 downto 0);
  		  icnt_stall : in std_logic_vector(12 downto 0);
		  cache_stall: in std_logic_vector(12 downto 0);
        goal_in : in std_logic_vector(left_size downto 0);
        speed_in : in sfixed(left_size downto 0);
        power_in : in sfixed(8 downto 0);
        actout0 : out std_logic_vector(2 downto 0);
        actout1 : out std_logic_vector(2 downto 0);
		  wavefront : out std_logic_vector(5 downto 0)        
);
end control;

architecture behavioral of control is

type gpu_control_record is
  record
     speed : sfixed(4 downto -9);
     power: sfixed(8 downto -4);
  end record;
type gpu_control_array is array (0 to 7) of gpu_control_record; 

signal counter : std_logic;

begin

process(clk,reset)
variable gpu_control_state : gpu_control_array;
variable freq_state : gpu_control_array;

variable last_speedup : sfixed(4 downto -9);

variable current_speed :  sfixed(left_size downto -9);
variable current_power :  sfixed(8 downto -9);
variable speed_goal : sfixed(left_size downto 0);
variable last_act0, last_act1 : integer;

variable max_speedup, min_speedup : sfixed(4 downto -9);
variable var_wavefront, var_dram_stall, var_icnt_stall, var_cache_stall : integer;

variable error,   p_minus, eo : sfixed(left_size downto -9);
variable  u, uo : sfixed(4 downto -9);
variable qw, x_hat_minus, p, x_hat, k, k_buf1, k_buf2, b_speed : sfixed(5 downto -9);
variable freq1 : sfixed(3 downto 0);
variable cost1, prop_speed : sfixed(8 downto -9);

begin
    if (reset = '1') then
        actout0 <= "000";
        actout1 <= "000";
		  counter <= '1';
        
        gpu_control_state(0).speed  := to_sfixed(1.0,4, -9);
        gpu_control_state(1).speed  := to_sfixed(2.0,4, -9);
        gpu_control_state(2).speed  := to_sfixed(3.0,4, -9);
        gpu_control_state(3).speed  := to_sfixed(4.0,4, -9);
        gpu_control_state(4).speed  := to_sfixed(5.0,4, -9);
        gpu_control_state(5).speed  := to_sfixed(6.0,4, -9);
        gpu_control_state(6).speed  := to_sfixed(7.0,4, -9);
        gpu_control_state(7).speed  := to_sfixed(8.0,4, -9);

        
        gpu_control_state(0).power  := to_sfixed(1.0,8, -4);
        gpu_control_state(1).power  := to_sfixed(1.2,8, -4);
        gpu_control_state(2).power  := to_sfixed(1.4,8, -4);
        gpu_control_state(3).power  := to_sfixed(1.6,8, -4);
        gpu_control_state(4).power  := to_sfixed(1.8,8, -4);
        gpu_control_state(5).power  := to_sfixed(1.9,8, -4);
        gpu_control_state(6).power  := to_sfixed(2.2,8, -4);
        gpu_control_state(7).power  := to_sfixed(2.5,8, -4);
		  
	  
		  freq_state(0).power  := to_sfixed(1.0,8, -4);
        freq_state(1).power  := to_sfixed(1.2,8, -4);
        freq_state(2).power  := to_sfixed(1.4,8, -4);
        freq_state(3).power  := to_sfixed(1.6,8, -4);
        freq_state(4).power  := to_sfixed(1.8,8, -4);
        freq_state(5).power  := to_sfixed(1.9,8, -4);
        freq_state(6).power  := to_sfixed(2.2,8, -4);
        freq_state(7).power  := to_sfixed(2.4,8, -4);
		  
        last_act1 := 0;

        max_speedup := to_sfixed(0.0,max_speedup);
        min_speedup := to_sfixed(100.0,max_speedup);
        last_speedup := to_sfixed(1.0,last_speedup);
        x_hat := to_sfixed(1.0, x_hat);



    
    p_minus := to_sfixed(1.0, p_minus);
    p := to_sfixed(0.0, p);
    uo := to_sfixed(0.0,uo);
    eo := to_sfixed(0.0, eo);
    cost1 := to_sfixed(10000.0,cost1);
	 prop_speed := to_sfixed(10000.0,prop_speed);


    current_speed := to_sfixed(1.0, current_speed);
	 current_power := to_sfixed(1.0, current_power);
	 var_wavefront := 48;
        
    elsif (clk'event and clk = '1') then
	     
    current_speed(left_size downto 0) := speed_in;
    current_power(8 downto 0) := power_in;
	 var_dram_stall := conv_integer(unsigned(dram_stall));
	 var_icnt_stall := conv_integer(unsigned(icnt_stall));
	 var_cache_stall := conv_integer(unsigned(cache_stall));
	    	 
	 speed_goal := to_sfixed(goal_in, speed_goal);
    error := resize(speed_goal-current_speed, error);
	 
	 speed_goal := resize(speed_goal + error, speed_goal);
    
      last_speedup := resize((gpu_control_state(last_act0).speed  * gpu_control_state(last_act1).speed),last_speedup);
      x_hat_minus := x_hat;
      p_minus := resize((p+0.01), p_minus);
      k_buf1 := resize((last_speedup*p_minus * last_speedup + 0.01), k_buf1);
      k_buf2 := resize((p_minus * last_speedup), k_buf2);
      k := resize((k_buf2/k_buf1), k);
      x_hat := resize((x_hat_minus + k * (current_speed-(last_speedup*x_hat_minus))), x_hat);
      p := resize(((1-(k*last_speedup))*p_minus), p);
      b_speed := resize((1.0/x_hat), b_speed);
                
	 gpu_control_state(last_act1).speed := resize((0.85*current_speed*b_speed*(8/(last_act1+1)))+(0.15*gpu_control_state(last_act1).speed) ,4, -9);
    
	 if last_act1 /= 7 then
		 if gpu_control_state(last_act1).speed > gpu_control_state(last_act1+1).speed then
		 gpu_control_state(last_act1+1).speed := gpu_control_state(last_act1).speed;
		 end if;
    end if;	 
	 
	 gpu_control_state(last_act1).power := resize((0.85*current_power/freq_state(last_act0).power)+(0.15*gpu_control_state(last_act1).power) ,8, -4);
    
    u :=  resize( (uo +  b_speed * error ), u);  

	 if(var_cache_stall > 1024) then
		var_wavefront := var_wavefront - 4;
		if (var_wavefront > 32) then
		var_wavefront := 32;
		end if;		
	 else 
		var_wavefront := var_wavefront + 4;
	 end if;
	 
		if (var_wavefront > 48) then
		var_wavefront := 48;
		end if;
		if (var_wavefront < 4) then
		var_wavefront := 4;
		end if;		
		
    prop_speed := to_sfixed(1000,prop_speed);
    for i in 0 to 7 loop
        if(gpu_control_state(i).speed > max_speedup)then
          max_speedup := gpu_control_state(i).speed;
        end if;
        if(gpu_control_state(i).speed < min_speedup)then
          min_speedup := gpu_control_state(i).speed; 
        end if;
	 end loop;
	 
	 if(u<min_speedup) then 
    u := min_speedup;
    end if;
    
      if(u>max_speedup) then
      u :=resize(max_speedup,u);
      end if;
    
	 if(var_dram_stall > var_icnt_stall) then
	 for i in 6 to 7 loop
	  if(gpu_control_state(i).speed > u)then
         freq1 := resize((u*8/gpu_control_state(i).speed), freq1);
         last_act0 := to_integer(freq1);	
			cost1 := resize(freq_state(last_act0).power * gpu_control_state(i).power ,cost1);	
		  if(cost1 < prop_speed)then
		  prop_speed := cost1;
		  last_act1 := i;
		  end if;
     end if;
	 end loop; 
	 else
	 for i in 0 to 5 loop
	  if(gpu_control_state(i).speed > u)then
         freq1 := resize((u*8/gpu_control_state(i).speed), freq1);
         last_act0 := to_integer(freq1);	
			cost1 := resize(freq_state(last_act0).power * gpu_control_state(i).power ,cost1);	
		  if(cost1 < prop_speed)then
		  prop_speed := cost1;
		  last_act1 := i;
		  end if;
     end if;
	end loop; 
	end if;
	
  actout0 <= conv_std_logic_vector(last_act0, 3);
  actout1 <= conv_std_logic_vector(last_act1, 3);
  wavefront <= conv_std_logic_vector(var_wavefront, 6);
  uo := u;
  eo := error;
 end if;
  

end process;

end behavioral;