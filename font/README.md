# Console font

> Originally this font was based on a dump of a BBC Micro font. Extended with
> box drawing characters from font image downloaded from
> https://dffd.bay12games.com/file.php?id=408

This font is used by the console and mimics the font used on the BBC Micro
released in the UK by Acorn Computers.

The console font is laid out in memory in a manner a bit different from usual in
order to optimise access when rendering. Each 8x8 character is represented as 8
bytes with each row represented by one byte. The MSB bit is the left-most pixel.
Unusually a "1" bit represents black and "0" represents white. Rather than being
laid out in memory as 8 bytes for character 0, 8 for character 1, etc. all of
the first rows for each character are in the first 256 bytes, then row 1, etc.

The [mkfont.py](mkfont.py) script will convert the [PNG
representation](beeb.png) of the font into a file suitable for use by the
console driver.

To run the script:

```console
$ pip3 install --user virtualenv  # only if you don't have virtualenv installed
$ python3 -m virtualenv -p $(which python3) venv
$ . venv/bin/activate
$ pip3 install -r mkfont-requirements.txt
$ python3 mkfont.py beeb.png ../tvout/font.cpp
```
