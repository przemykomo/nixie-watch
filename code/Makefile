SRC = i2c/i2c-master.c main.c
TARGET = output
LIB = -Ii2c/

clean:
	rm $(TARGET).elf $(TARGET).hex

attiny: $(SRC)
	avr-gcc -Os -mmcu=attiny25 -DF_CPU=1000000 $(LIB) $(SRC) -o $(TARGET).elf
	avr-objcopy -O ihex $(TARGET).elf $(TARGET).hex

flash_attiny:
	avrdude -c usbasp -p attiny25 -U flash:w:$(TARGET).hex
