SRC = i2c/i2c-master.c main.c
TARGET = output
LIB = -Ii2c/

clean:
	rm $(TARGET).elf $(TARGET).hex

attiny: $(SRC)
	#avr-gcc -Os -mmcu=attiny45 -DF_CPU=8000000 -DI2C_LOW_LEVEL_API $(LIB) $(SRC) -o $(TARGET).elf
	avr-gcc -Os -mmcu=attiny45 -DF_CPU=8000000 $(LIB) $(SRC) -o $(TARGET).elf
	avr-objcopy -O ihex $(TARGET).elf $(TARGET).hex

flash_attiny:
	avrdude -c usbasp -p attiny45 -U flash:w:$(TARGET).hex
