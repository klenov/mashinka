#!/usr/bin/ruby
#(http://rubygems.org/gems/serialport)
 
require 'serialport'
require 'fcntl'
require 'io/wait'
require 'logger'

log = Logger.new('log/read_serial.log')
# FATAL or ERROR for normal use
log.level = Logger::DEBUG
log.info 'Programm started.'

pshyk_duration = 200.0 # miliseconds
PSHYK_BYTE_TO_SEND = ( (pshyk_duration/333).round + 32 ).chr

# ищем первый тти
i = 0
port_str = false
until port_str
  if File.exists?("/dev/ttyUSB#{i}") || i > 10
     port_str = "/dev/ttyUSB#{i}"
     log.info  "/dev/ttyUSB#{i} "
  end
  i += 1
end

baud_rate = 9600
data_bits = 8
stop_bits = 1
parity = SerialPort::NONE
 
sp = SerialPort.new(port_str, baud_rate, data_bits, stop_bits, parity)

# нужно для маленького юсб-ттл конвертора
#sp.rts = 0

# не ждать , читать только то что доступно
sp.read_timeout = -1


Thread.new do
  loop do

    s = gets.chomp
    log.debug s + '(recived from parent)'

    if s == "pshyk"
        sp.write(PSHYK_BYTE_TO_SEND)
        log.debug "pshyk, duration = " + PSHYK_BYTE_TO_SEND
      elsif s == "stop_pshyk"
        # any byte beetwen 96 ('`') and 128, for instance 'a', 'b', ...
        log.debug "stop_pshyk"
        sp.write('a')
      elsif s == 'forward'
        sp.write("\xA3")
      elsif s == 'backward'
        sp.write("\xA4")
      elsif s == 'left'
        sp.write("\xA1")
      elsif s == 'right'
        sp.write("\xA2")
      elsif s == 'stop'
        sp.write("\xA5")
      elsif s == 'toggle_p_mode'
        sp.write("\x81")
      elsif s == 'led_inc'
        sp.write("\x82") 
      elsif s == "check_connection"
        log.debug "check_connection"
        rndm = Random.new
        random_byte = rndm.rand(224..253)
        
         while sp.ready? do
          sp.getc
          log.debug "read some data to clear serial buffer before random_byte sending"
         end
        
        sp.write(random_byte.chr)

        log.debug "sended byte: " + random_byte.to_s
        sleep(1)
        if sp.ready?
          answer = sp.getc
          log.debug "recieved byte: " + answer.ord.to_s
          if answer.ord + 224 == random_byte # так как я отправляю байт 111XXXXX, а получаю 000XXXXX
            puts 'connection_ok'
          end 
        end
      end

  end
end


loop do
  sleep 1
end
 
sp.close 



