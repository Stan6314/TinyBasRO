Memory content for the TinyBasRO microcomputer
==============================================

Here you will find two compiled versions of the program for the TinyBasRO microcomputer, which can be loaded directly into the microcontroller memory using the WCHLinkE programmer.

The file TinyBasRO.hex can be loaded into the CH32V003 memory with the standard display orientation (the display connector with the I2C bus is on top). If you would like to rotate the orientation of the microcomputer display (the display will be readable with the I2C bus connector on the bottom), use the file TinyBasRO_upsidedown.hex.

Notice!
=======

Be careful when reading and downloading HEX files from GitHub. When downloading a file, the end of the line is truncated to a single LF character (Linux-style). However, WCH-LinkUtility requires Windows-style line termination - CR+LF! When loading a file into its buffer, WCH-LinkUtility will only load one line of HEX file!

WCH-LinkUtility then acts as if it programmed the entire memory, but only 16 bytes are written. That is why the HEX files are now saved in .zip format.


* * *

Obsah paměti pro mikropočítač TinyBasRO
=======================================

Zde naleznete přeložené dvě verze programu pro mikropočítač TinyBasRO, které je možné nahrát přimo do paměti mikrořadiče pomocí programátoru WCHLinkE.

Soubor TinyBasRO.hex je možné nahrát do paměti CH32V003 při standardní orientaci displeje (konektor displeje s I2C sběrnicí je nahoře). Pokud byste chtěli orientaci displeje mikropočítače otočit (displej bude čitelný při konektoru I2C sběrnice dole) použijte soubor TinyBasRO_upsidedown.hex.

Upozornění!
===========

Dejte si pozor na čtení a stahování HEX souborů z GitHub. Při stažení souboru je konec řádku zkrácen na jeden znak LF (jako v Linuxu). WCH-LinkUtility ale vyžaduje ukončení řádku ve stylu Windows - CR+LF! Při načítání souboru do svého bufferu pak WCH-LinkUtility načte pouze jeden řádek HEX souboru!

WCH-LinkUtility pak fungují jakoby naprogramovaly celou paměť, ale zapíše se pouze 16 bajtů. Proto jsou HEX soubory nyní uloženy v .zip formátu.

