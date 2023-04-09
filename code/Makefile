SRC	= main.c
TARGET	= output

clean:
	rm $(TARGET).elf $(TARGET).hex

attiny: $(SRC)
	avr-gcc -Os -mmcu=attiny45 -DF_CPU=1000000 $(SRC) -o $(TARGET).elf
	avr-objcopy -O ihex $(TARGET).elf $(TARGET).hex

flash_attiny:
	avrdude -c usbasp -p attiny45 -U flash:w:$(TARGET).hex
