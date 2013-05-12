#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2.cv as cv

# для асинхронной коммуникации с сериал-портом
import sys, pexpect

#import random # генерация номера для проверки коммуникации

# в питоне нет констант? o_O

# сделать по кнопке увеличение уменьшение размера точки
# сделать программно или аппаратно кнопку для пшика

# может надо отслеживать сколько он пшикал в каждом месте?

"""
# for serial
import fcntl
import struct
import sys
import termios
import time
"""

"""
At my work I'm communicating with an embedded device via a serial-to-USB cable. It works fine, except for one little detail: when the "Ready To Send" (RTS) pin is high, the device shuts down. This is by design, but annoying nonetheless, since almost all serial communication software sets this pin to high. I've written a little Python program that opens the serial connection, sets the RTS pin to low, and shows all data read from it.

You can read the details after the break.

I've tested my solution on Linux, not sure if it works on other systems, as I'm using rather low-level calls to get the RTS pin low. Please leave a comment when you've tried!

The Python code defines two functions; one to set the RTS pin, and one to copy the data from the serial device to stdout. Here is the full code:
"""


def is_rect_nonzero(r):
    (_,_,w,h) = r
    return (w > 0) and (h > 0)

"""
# for serial    
def set_rts(fileobj, on_off):
    # Get current flags
    p = struct.pack('I', 0)
    flags = fcntl.ioctl(fileobj.fileno(), termios.TIOCMGET, p)

    # Convert four byte string to integer
    flags = struct.unpack('I', flags)[0]

    if on_off:
        flags |= termios.TIOCM_RTS
    else:
        flags &= ~termios.TIOCM_RTS

    # Set new flags
    p = struct.pack('I', flags)
    fcntl.ioctl(fileobj.fileno(), termios.TIOCMSET, p)    

def serial_init():
    device = '/dev/ttyUSB0'
    print 'Device: %s' % device
    serial_port = open(device, 'w')
    set_rts(serial_port, False)
    return serial_port;
"""


class Mashinka:

    def __init__(self):
        self.capture = cv.CaptureFromCAM(0)
        # понадобилось для ps3 eye установить  разрешение видео, иначе 320_240
        cv.SetCaptureProperty(self.capture, cv.CV_CAP_PROP_FRAME_WIDTH,  640)
        cv.SetCaptureProperty(self.capture, cv.CV_CAP_PROP_FRAME_HEIGHT, 480)
        cv.SetCaptureProperty(self.capture, cv.CV_CAP_PROP_FPS,          60)

        cv.NamedWindow( "Mashinka", 1 )
        cv.NamedWindow( "Histogram", 1 )
        # окно для отображения приблизительного хода рисования
        #cv.NamedWindow( "Painting", 1 )
        cv.SetMouseCallback( "Mashinka", self.on_mouse)

        self.drag_start = None      # Set to (x,y) when mouse starts drag
        self.track_window = None    # Set to rect when the mouse drag finishes
        
        # лучше использовать руби-скрипт: там есть тайминг для чтения сериал-порта
        self.child = pexpect.spawn("./read_serial.rb")
	self.child.sendline('Initial string.')       
        
        # очистка консоли? это жлобство

        print( "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\nKeys:\n"
            "    ESC - quit the program\n"
            "    b - switch to/from backprojection view\n"
            "To initialize tracking, drag across the object with the mouse\n\n\n\n\n\n" )
            
        # загрузка изображения ("трафарета") для рисования
        #self.stencil = cv.LoadImageM("images/line.png")   
        
        # координаты маркера
        self.x = 0
        self.y = 0
        
        # добавить сообщение что картинка загружена
        # можно принимать имя картинки как параметр AddWeighted
        # self.stencil = cv.LoadImageM("images/line.png", cv.CV_LOAD_IMAGE_GRAYSCALE)
        self.stencil = cv.LoadImage("images/line2.png")
        
        # размер (в пикселях камеры) точки нарисованной баллончиком
        self.dot_size = 10

        self.throttle = 0

        self.led_brightness = 0

        self.last_sended_command = ""
        self.connection_is_ok = False

        # пока  решил сам трафарет использовать как массив незакрашенных пикселей и модифицировать его
        #self.painted = self.stencil

    def hue_histogram_as_image(self, hist):
        """ Returns a nice representation of a hue histogram """

        histimg_hsv = cv.CreateImage( (320,200), 8, 3)
        
        mybins = cv.CloneMatND(hist.bins)
        cv.Log(mybins, mybins)
        (_, hi, _, _) = cv.MinMaxLoc(mybins)
        cv.ConvertScale(mybins, mybins, 255. / hi)

        w,h = cv.GetSize(histimg_hsv)
        hdims = cv.GetDims(mybins)[0]
        for x in range(w):
            xh = (180 * x) / (w - 1)  # hue sweeps from 0-180 across the image
            val = int(mybins[int(hdims * x / w)] * h / 255)
            cv.Rectangle( histimg_hsv, (x, 0), (x, h-val), (xh,255,64), -1)
            cv.Rectangle( histimg_hsv, (x, h-val), (x, h), (xh,255,255), -1)

        histimg = cv.CreateImage( (320,200), 8, 3)
        cv.CvtColor(histimg_hsv, histimg, cv.CV_HSV2BGR)
        return histimg

    def on_mouse(self, event, x, y, flags, param):
        if event == cv.CV_EVENT_LBUTTONDOWN:
            self.drag_start = (x, y)
        if event == cv.CV_EVENT_LBUTTONUP:
            self.drag_start = None
            self.track_window = self.selection
        if self.drag_start:
            xmin = min(x, self.drag_start[0])
            ymin = min(y, self.drag_start[1])
            xmax = max(x, self.drag_start[0])
            ymax = max(y, self.drag_start[1])
            self.selection = (xmin, ymin, xmax - xmin, ymax - ymin)
         # обработка нажатий правой кнопки для указания границ (левая верхняя, правая нижняя) прямоугольника для рисования
        if event == cv.CV_EVENT_RBUTTONUP:
			print('right button ');
			#self.paint_rectangle_sp = (x,y)
			#self.paint_rectangle_ep = 
			# вывод в консоль что точки установлены и замер прямоугольника после второй точки
			pass
				

    def run(self):
        hist = cv.CreateHist([180], cv.CV_HIST_ARRAY, [(0,180)], 1 )
        backproject_mode = False
        
        #for serial
        #serial_port = serial_init()
        
        # commands variables
        get_ir_data = 0
        
        
        while True:
            frame = cv.QueryFrame( self.capture )

            # Convert to HSV and keep the hue
            hsv = cv.CreateImage(cv.GetSize(frame), 8, 3)
            cv.CvtColor(frame, hsv, cv.CV_BGR2HSV)
            self.hue = cv.CreateImage(cv.GetSize(frame), 8, 1)
            cv.Split(hsv, self.hue, None, None, None)

            # Compute back projection
            backproject = cv.CreateImage(cv.GetSize(frame), 8, 1)

            # Run the cam-shift
            cv.CalcArrBackProject( [self.hue], backproject, hist )
            if self.track_window and is_rect_nonzero(self.track_window):
                crit = ( cv.CV_TERMCRIT_EPS | cv.CV_TERMCRIT_ITER, 10, 1)
                (iters, (area, value, rect), track_box) = cv.CamShift(backproject, self.track_window, crit)
                self.track_window = rect

            # If mouse is pressed, highlight the current selected rectangle
            # and recompute the histogram

            if self.drag_start and is_rect_nonzero(self.selection):
                sub = cv.GetSubRect(frame, self.selection)
                save = cv.CloneMat(sub)
                cv.ConvertScale(frame, frame, 0.5)
                cv.Copy(save, sub)
                x,y,w,h = self.selection
                cv.Rectangle(frame, (x,y), (x+w,y+h), (255,255,255))

                sel = cv.GetSubRect(self.hue, self.selection )
                cv.CalcArrHist( [sel], hist, 0)
                (_, max_val, _, _) = cv.GetMinMaxHistValue( hist)
                if max_val != 0:
                    cv.ConvertScale(hist.bins, hist.bins, 255. / max_val)
            elif self.track_window and is_rect_nonzero(self.track_window):
                cv.EllipseBox( frame, track_box, cv.CV_RGB(255,0,0), 3, cv.CV_AA, 0 )
                #получаем x, y объекта
                self.x = int( round(  track_box[0][0] ) )
                self.y = int( round(  track_box[0][1] ) )
                
                l = ""
                # тут надо добавить try или как-то рещить с таймингом
                # тут застревает если нет данных с сериала
                #c = serial_port.read(1)                                

                if get_ir_data:
                  # читаем не более 1024 байтов только если они уже есть в буфере, если в буфере ничего нет то вызывается ошибка pexpect.TIMEOUT
                  #self.child.read_nonblocking(size = 1024, timeout = 0)

                  # как отправлять данные?
                  #child.send('a')                                               
                  if c:
					  l = '; l = ' + str( ord( c ) )
					  
				# вывод координат и расстояния	  
                msg = ' x = ' + str( self.x ) + ' y = ' + str( self.y ) + l
                
                sys.stdout.write('\r' + ' '*len(msg)); sys.stdout.flush()
                
                sys.stdout.write(msg); sys.stdout.flush()
                
                # рисуем картинку поверх видеокадра
                #(src1, alpha, src2, beta, gamma, dst)
                #frame_gs = cv.CloneMatND(frame)
                
                #cv.CvtColor(frame, frame_gs, cv.CV_BGR2GRAY)
                #print frame.nChannels
                #print frame.depth
                #print frame.width
                #print frame.height
                
                #print self.stencil.nChannels
                #print self.stencil.depth
                
                #print type(self.stencil)                                
                
                frame_2 = cv.CloneImage(frame)

                if cv.Get2D(self.stencil, self.y, self.x)[0] > 0:      
                  #serial_port.write('$') 
		  # если не работает пересылка сообщений в руби-скрипт значит в руби-скрипте ошибка, так уже было несколько раз        
                  self.child.sendline('pshyk')
                  self.last_sended_command = 'pshyk'
                  cv.Circle(self.stencil, (self.x, self.y), self.dot_size, cv.CV_RGB(0,0,0), -1) 
                else:
                  # если последняя команда была "pshyk", то посылаем "stop_pshyk"
                  if self.last_sended_command == 'pshyk':
                    self.last_sended_command = 'stop_pshyk'
                    self.child.sendline('stop_pshyk')
                 # print 'draw'
                
                cv.AddWeighted(frame, 1, self.stencil, 0.2, 0, frame_2)                
                frame = frame_2
                
                #print()
            # тут можно написать отрисовку интерфейса
            # сначала рисуем прямоугольник, на который будем выводить текст
            # сделать несколько прямоугольников которые могут быть красными и зелеными, типа проверено ли соединение, в каком стостоянии батарея

            pt1 =  (0, 440)
            pt2 =  (640, 480)
            cv.Rectangle(frame, pt1, pt2, 0, cv.CV_FILLED)
            
            font = cv.InitFont(cv.CV_FONT_HERSHEY_PLAIN, 1, 1, 0, 1)

            cv.PutText(frame, "Connection:", (20, 460), font, cv.CV_RGB( 255, 255, 255 ))
            if self.connection_is_ok:            
              cv.PutText(frame, "OK", (125, 460), font, cv.CV_RGB( 30, 255, 10 ))
            else:
              cv.PutText(frame, "?",  (125, 460), font, cv.CV_RGB( 255, 10, 10 ))  

            cv.PutText(frame, "Dot size: " + str(self.dot_size), (155, 460), font, cv.CV_RGB( 255, 255, 255 ))  

            cv.PutText(frame, "Throttle: " + str(self.throttle) + ' %', (295, 460), font, cv.CV_RGB( 255, 255, 255 ))  

            cv.PutText(frame, "Led: " + str(self.led_brightness) + ' %', (450, 460), font, cv.CV_RGB( 255, 255, 255 ))  

            if not backproject_mode:
                cv.ShowImage( "Mashinka", frame )
            else:
                cv.ShowImage( "Mashinka", backproject)
            # закомментировать для отключения гистограммы
            cv.ShowImage( "Histogram", self.hue_histogram_as_image(hist))

            c = cv.WaitKey(7) % 0x100
            #здесь обработка нажатия других клавиш
            if c == 27:
                self.child.close()
                break
            elif c == ord("c") or c == ord("C"):
		# тут предпологается добавить обработку проверки связи с ардуинкой
		
		# тут я читаю все что прислал руби скрипт до тех пор пока не произойдет исключение "таймаут", чтобы очистить буфер
		while True:
                  try:
                    self.child.read_nonblocking(size = 1024, timeout = 0)
                  except (pexpect.TIMEOUT, pexpect.EOF):
                    break
		
                print("Attempt to check connection...")
                self.child.sendline('check_connection')
                # в руби-скрипте генерируем случайное число и посылаем на сериал и ждем ответа
                # а тут надо возможно очистить стдин и ждать ответ и вывести сообщение
                # хорошо бы в скобках выводить отправленный байт
                
                try:
                  s = self.child.read_nonblocking(size = 1024, timeout = 0) + self.child.read_nonblocking(size = 1024, timeout = 2)
                except (pexpect.TIMEOUT, pexpect.EOF):
                  s = ''
                

                # почему строка именно такая, а именно откуда берется check_connection я не понял, почему-то функция read возвращает все строки отправленные child
                if s and s == 'check_connection\r\nconnection_ok\r\n':
                  print('connection is ok');
                  self.connection_is_ok = true
                
            elif c == ord("b"):
                backproject_mode = not backproject_mode
            elif c == ord("w"):   
                self.child.sendline('forward')
            elif c == ord("s"):   
                self.child.sendline('backward')
            elif c == ord("a"):   
                self.child.sendline('left')
            elif c == ord("d"):   
                self.child.sendline('right')
            elif c == ord("f"):   
                self.child.sendline('stop')

if __name__=="__main__":
    demo = Mashinka()
    demo.run()
    
    # завершить руби-скрипт!
    cv.DestroyAllWindows()
    