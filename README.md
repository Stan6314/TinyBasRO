Microcomputer TinyBasRO
======================

![screenshot](TinyBasRO.jpg)

The TinyBasRO microcomputer is a very simple device that even an electrical engineering beginner should be able to assemble and put into operation. It resembles simple 8-bit computers from the 1980s. After assembly, it can be programmed very easily using the Tiny BASIC language, which should be manageable even for those who have never programmed before. The basis is a small 32-bit RISC-V processor cooperating with serial I2C memory, where the user-created program is stored.

A microcomputer is suitable for the first introduction to computing technology when teaching hardware and software. It can be used not only for educational purposes, but can also be used as a control element for simple electronic devices.

Technical data for TinyBasRO
----------------------------

*   TinyBasHW_KiCad - HW documents (schematic and printed circuit board) in KiCad 9.0 format
*   src - Source files for TinyBasRO for CH32V003 created in ch32fun development environment
*   Bin - Compiled binary content in HEX format for direct loading into CH32V003 memory
*   Manuals - Manuals for HW, Tiny BASIC programming language and its examples and advice for beginners
*   EEPROM_examples - Contents of serial EEPROM memory in HEX format with examples of 8 ready-made Tiny BASIC sample programs for loading into microcomputer memory

Requirements for building and operating a TinyBasRO
---------------------------------------------------

*   Printed circuit board
*   Electrical components according to the list in the TinyBasRO_HW manual
*   WCHLinkE programmer for programming CH32V003
*   OLED display 128x128 with SH1107 driver
*   Keyboard CardKB from M5STACK
*   5V power supply

Using a microcomputer
---------------------

The assembled microcomputer only requires a 5 V power supply, OLED display 128x128 with SH1107 driver and CardKB keyboard. The board has several interfaces for connecting expansion modules via the I2C bus or controlled elements via the input/output connector. In addition, the TinyBasRO allows you to easily light up a string of up to 100 WS2812 LEDs compared to computer TinyBasRV.

![screenshot](Perif.jpg)

After debugging the program, the TinyBasRO microcomputer can work independently without a monitor and keyboard and control, for example, some LED lighting effects.

* * *

Mikropočítač TinyBasRO
======================

![screenshot](TinyBasRO.jpg)

Mikropočítač TinyBasRO je velmi jednoduché zařízení, které by měl dokázat sestavit a uvést do provozu i elektrotechnický začátečník. Podobá se jednoduchým 8bitovým počítačům z 80tých let. Po sestavení jej lze velmi jednoduše programovat pomocí jazyka Tiny BASIC, který by měl zvládnout i ten, kdo dříve neprogramoval. Základem je malý 32bitový RISC-V procesor spolupracující se sériovou I2C pamětí, kam se ukládá uživatelem tvořený program.

Mikropočítač je vhodný pro první seznámení s výpočetní technikou při výuce technického i programového vybavení. Může být použit nejen pro výukové účely, ale lze jej nasadit jako řídicí prvek pro jednoduché elektronické přístroje.

Technické podklady pro TinyBasRO
--------------------------------

*   TinyBasHW_KiCad - HW podklady (schéma a plošný spoj) ve formátu programu KiCad 9.0
*   src - Zdrojové soubory k TinyBasRO pro CH32V003 vytvořené v prostředí ch32fun
*   Bin - Přeložený binární obsah ve formátu HEX pro přímé nahrání do paměti CH32V003
*   Manuals - Manuály pro HW, programovací jazyk Tiny BASIC a jeho příklady a rady pro začátečníky
*   EEPROM_examples - Obsah sériové EEPROM paměti ve formátu HEX s příklady 8 hotových ukázkových programů Tiny BASIC pro nahrání do paměti mikropočítače

Požadavky pro stavbu a provoz TinyBasRO
---------------------------------------

*   Deska plošného spoje
*   Elektrotechnické součástky podle seznamu v manuálu TinyBasRO_HW
*   Programátor WCHLinkE pro naprogramování CH32V003
*   OLED displej 128x128 s řadičem SH1107
*   Klávesnice CardKB od M5STACK
*   Napájecí zdroj 5 V

Použití mikropočítače
---------------------

Sestavený mikropočítač vyžaduje pouze napájecí zdroj 5 V, OLED displej 128x128 s řadičem SH1107 a klávesnici CardKB. Na desce je několik rozhraní pro připojení rozšiřujících modulů přes I2C sběrnici nebo ovládaných členů přes konektor vstupů/výstupů. Navíc proti počítači TinyBasRV umožňuje snadno rozsvěcovat řetězec až se 100 LED WS2812. 

![screenshot](Perif.jpg)

Po odladění programu, může mikropočítač TinyBasRO pracovat samostatně bez monitoru a klávesnice a řídit např. nějaké světelné LED efekty.

