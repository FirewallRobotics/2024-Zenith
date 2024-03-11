import unicornhat as UH 
from bitarray import bitarray
import time
import os
import socket
import ntcore

# Letter space
x0= bitarray('00000000')
x1= bitarray('00000000')
x2= bitarray('00000000')
x3= bitarray('00000000')
x4= bitarray('00000000')
x5= bitarray('00000000')
x6= bitarray('00000000')
x7= bitarray('00000000')
letter_space=[x0,x1,x2,x3,x4,x5,x6,x7]

# Letter A
x0= bitarray('00000000')
x1= bitarray('00111100')
x2= bitarray('00100100')
x3= bitarray('00111100')
x4= bitarray('00100100')
x5= bitarray('00100100')
x6= bitarray('00100100')
x7= bitarray('00000000')
letter_A=[x0,x1,x2,x3,x4,x5,x6,x7]

# Letter B
x0= bitarray('00000000')
x1= bitarray('00111000')
x2= bitarray('00100100')
x3= bitarray('00111000')
x4= bitarray('00100100')
x5= bitarray('00100100')
x6= bitarray('00111000')
x7= bitarray('00000000')
letter_B=[x0,x1,x2,x3,x4,x5,x6,x7]

# Letter C
x0= bitarray('00000000')
x1= bitarray('00111100')
x2= bitarray('00100000')
x3= bitarray('00100000')
x4= bitarray('00100000')
x5= bitarray('00100000')
x6= bitarray('00111100')
x7= bitarray('00000000')
letter_C=[x0,x1,x2,x3,x4,x5,x6,x7]

# Letter D
x0= bitarray('00000000')
x1= bitarray('00111000')
x2= bitarray('00100100')
x3= bitarray('00100100')
x4= bitarray('00100100')
x5= bitarray('00100100')
x6= bitarray('00111000')
x7= bitarray('00000000')
letter_D=[x0,x1,x2,x3,x4,x5,x6,x7]

# Letter E
x0= bitarray('00000000')
x1= bitarray('00111100')
x2= bitarray('00100000')
x3= bitarray('00111100')
x4= bitarray('00100000')
x5= bitarray('00100000')
x6= bitarray('00111100')
x7= bitarray('00000000')
letter_E=[x0,x1,x2,x3,x4,x5,x6,x7]

# Letter F
x0= bitarray('00000000')
x1= bitarray('00111100')
x2= bitarray('00100000')
x3= bitarray('00111100')
x4= bitarray('00100000')
x5= bitarray('00100000')
x6= bitarray('00100000')
x7= bitarray('00000000')
letter_F=[x0,x1,x2,x3,x4,x5,x6,x7]

# Letter G
x0= bitarray('00000000')
x1= bitarray('00111100')
x2= bitarray('00100000')
x3= bitarray('00100000')
x4= bitarray('00101100')
x5= bitarray('00100100')
x6= bitarray('00111100')
x7= bitarray('00000000')
letter_G=[x0,x1,x2,x3,x4,x5,x6,x7]

# Letter H
x0= bitarray('00000000')
x1= bitarray('00100100')
x2= bitarray('00100100')
x3= bitarray('00111100')
x4= bitarray('00100100')
x5= bitarray('00100100')
x6= bitarray('00100100')
x7= bitarray('00000000')
letter_H=[x0,x1,x2,x3,x4,x5,x6,x7]

# Letter I
x0= bitarray('00000000')
x1= bitarray('00011100')
x2= bitarray('00001000')
x3= bitarray('00001000')
x4= bitarray('00001000')
x5= bitarray('00001000')
x6= bitarray('00011100')
x7= bitarray('00000000')
letter_I=[x0,x1,x2,x3,x4,x5,x6,x7]

# Letter J
x0= bitarray('00000000')
x1= bitarray('00011100')
x2= bitarray('00001000')
x3= bitarray('00001000')
x4= bitarray('00001000')
x5= bitarray('00001000')
x6= bitarray('00011000')
x7= bitarray('00000000')
letter_J=[x0,x1,x2,x3,x4,x5,x6,x7]

# Letter K
x0= bitarray('00000000')
x1= bitarray('00100100')
x2= bitarray('00100100')
x3= bitarray('00101000')
x4= bitarray('00110000')
x5= bitarray('00101000')
x6= bitarray('00100100')
x7= bitarray('00000000')
letter_K=[x0,x1,x2,x3,x4,x5,x6,x7]

# Letter L
x0= bitarray('00000000')
x1= bitarray('00100000')
x2= bitarray('00100000')
x3= bitarray('00100000')
x4= bitarray('00100000')
x5= bitarray('00100000')
x6= bitarray('00111100')
x7= bitarray('00000000')
letter_L=[x0,x1,x2,x3,x4,x5,x6,x7]

# Letter M
x0= bitarray('00000000')
x1= bitarray('01111100')
x2= bitarray('01010100')
x3= bitarray('01010100')
x4= bitarray('01010100')
x5= bitarray('01010100')
x6= bitarray('01010100')
x7= bitarray('00000000')
letter_M=[x0,x1,x2,x3,x4,x5,x6,x7]

# Letter N
x0= bitarray('00000000')
x1= bitarray('00100100')
x2= bitarray('00100100')
x3= bitarray('00110100')
x4= bitarray('00101100')
x5= bitarray('00100100')
x6= bitarray('00100100')
x7= bitarray('00000000')
letter_N=[x0,x1,x2,x3,x4,x5,x6,x7]

# Letter O
x0= bitarray('00000000')
x1= bitarray('00111100')
x2= bitarray('00100100')
x3= bitarray('00100100')
x4= bitarray('00100100')
x5= bitarray('00100100')
x6= bitarray('00111100')
x7= bitarray('00000000')
letter_O=[x0,x1,x2,x3,x4,x5,x6,x7]

# Letter P
x0= bitarray('00000000')
x1= bitarray('00111100')
x2= bitarray('00100100')
x3= bitarray('00111100')
x4= bitarray('00100000')
x5= bitarray('00100000')
x6= bitarray('00100000')
x7= bitarray('00000000')
letter_P=[x0,x1,x2,x3,x4,x5,x6,x7]

# Letter Q
x0= bitarray('00000000')
x1= bitarray('00111100')
x2= bitarray('00100100')
x3= bitarray('00100100')
x4= bitarray('00100100')
x5= bitarray('00101100')
x6= bitarray('00111100')
x7= bitarray('00000000')
letter_Q=[x0,x1,x2,x3,x4,x5,x6,x7]

# Letter R
x0= bitarray('00000000')
x1= bitarray('00111100')
x2= bitarray('00100100')
x3= bitarray('00111100')
x4= bitarray('00110000')
x5= bitarray('00101000')
x6= bitarray('00100100')
x7= bitarray('00000000')
letter_R=[x0,x1,x2,x3,x4,x5,x6,x7]

# Letter S
x0= bitarray('00000000')
x1= bitarray('00111100')
x2= bitarray('00100000')
x3= bitarray('00111100')
x4= bitarray('00000100')
x5= bitarray('00000100')
x6= bitarray('00111100')
x7= bitarray('00000000')
letter_S=[x0,x1,x2,x3,x4,x5,x6,x7]

# Letter T
x0= bitarray('00000000')
x1= bitarray('00011100')
x2= bitarray('00001000')
x3= bitarray('00001000')
x4= bitarray('00001000')
x5= bitarray('00001000')
x6= bitarray('00001000')
x7= bitarray('00000000')
letter_T=[x0,x1,x2,x3,x4,x5,x6,x7]

# Letter U
x0= bitarray('00000000')
x1= bitarray('00100100')
x2= bitarray('00100100')
x3= bitarray('00100100')
x4= bitarray('00100100')
x5= bitarray('00100100')
x6= bitarray('00111100')
x7= bitarray('00000000')
letter_U=[x0,x1,x2,x3,x4,x5,x6,x7]

# Letter V
x0= bitarray('00000000')
x1= bitarray('00100100')
x2= bitarray('00100100')
x3= bitarray('00100100')
x4= bitarray('00100100')
x5= bitarray('00100100')
x6= bitarray('00011000')
x7= bitarray('00000000')
letter_V=[x0,x1,x2,x3,x4,x5,x6,x7]

# Letter W
x0= bitarray('00000000')
x1= bitarray('01010100')
x2= bitarray('01010100')
x3= bitarray('01010100')
x4= bitarray('01010100')
x5= bitarray('01010100')
x6= bitarray('01111100')
x7= bitarray('00000000')
letter_W=[x0,x1,x2,x3,x4,x5,x6,x7]

# Letter X
x0= bitarray('00000000')
x1= bitarray('00100100')
x2= bitarray('00100100')
x3= bitarray('00011000')
x4= bitarray('00011000')
x5= bitarray('00100100')
x6= bitarray('00100100')
x7= bitarray('00000000')
letter_X=[x0,x1,x2,x3,x4,x5,x6,x7]

# Letter Y
x0= bitarray('00000000')
x1= bitarray('00100100')
x2= bitarray('00100100')
x3= bitarray('00111100')
x4= bitarray('00000100')
x5= bitarray('00000100')
x6= bitarray('00111100')
x7= bitarray('00000000')
letter_Y=[x0,x1,x2,x3,x4,x5,x6,x7]

# Letter Z
x0= bitarray('00000000')
x1= bitarray('00111100')
x2= bitarray('00000100')
x3= bitarray('00001000')
x4= bitarray('00010000')
x5= bitarray('00100000')
x6= bitarray('00111100')
x7= bitarray('00000000')
letter_Z=[x0,x1,x2,x3,x4,x5,x6,x7]

# Number 0
x0= bitarray('00000000')
x1= bitarray('00111100')
x2= bitarray('00110100')
x3= bitarray('00100100')
x4= bitarray('00100100')
x5= bitarray('00101100')
x6= bitarray('00111100')
x7= bitarray('00000000')
number_0=[x0,x1,x2,x3,x4,x5,x6,x7]

# Number 1
x0= bitarray('00000000')
x1= bitarray('00011000')
x2= bitarray('00001000')
x3= bitarray('00001000')
x4= bitarray('00001000')
x5= bitarray('00001000')
x6= bitarray('00011100')
x7= bitarray('00000000')
number_1=[x0,x1,x2,x3,x4,x5,x6,x7]

# Number 2
x0= bitarray('00000000')
x1= bitarray('00111100')
x2= bitarray('00000100')
x3= bitarray('00000100')
x4= bitarray('00111000')
x5= bitarray('00100000')
x6= bitarray('00111100')
x7= bitarray('00000000')
number_2=[x0,x1,x2,x3,x4,x5,x6,x7]

# Number 3
x0= bitarray('00000000')
x1= bitarray('00111100')
x2= bitarray('00000100')
x3= bitarray('00000100')
x4= bitarray('00111100')
x5= bitarray('00000100')
x6= bitarray('00111100')
x7= bitarray('00000000')
number_3=[x0,x1,x2,x3,x4,x5,x6,x7]

# Number 4
x0= bitarray('00000000')
x1= bitarray('00100100')
x2= bitarray('00100100')
x3= bitarray('00111100')
x4= bitarray('00000100')
x5= bitarray('00000100')
x6= bitarray('00000100')
x7= bitarray('00000000')
number_4=[x0,x1,x2,x3,x4,x5,x6,x7]

# Number S
x0= bitarray('00000000')
x1= bitarray('00111100')
x2= bitarray('00100000')
x3= bitarray('00011100')
x4= bitarray('00000100')
x5= bitarray('00000100')
x6= bitarray('00111100')
x7= bitarray('00000000')
number_5=[x0,x1,x2,x3,x4,x5,x6,x7]

# Number 6
x0= bitarray('00000000')
x1= bitarray('00111100')
x2= bitarray('00100000')
x3= bitarray('00111100')
x4= bitarray('00100100')
x5= bitarray('00100100')
x6= bitarray('00111100')
x7= bitarray('00000000')
number_6=[x0,x1,x2,x3,x4,x5,x6,x7]

# Number 7
x0= bitarray('00000000')
x1= bitarray('00111100')
x2= bitarray('00000100')
x3= bitarray('00000100')
x4= bitarray('00000100')
x5= bitarray('00000100')
x6= bitarray('00000100')
x7= bitarray('00000000')
number_7=[x0,x1,x2,x3,x4,x5,x6,x7]

# Number 8
x0= bitarray('00000000')
x1= bitarray('00111100')
x2= bitarray('00100100')
x3= bitarray('00111100')
x4= bitarray('00100100')
x5= bitarray('00100100')
x6= bitarray('00111100')
x7= bitarray('00000000')
number_8=[x0,x1,x2,x3,x4,x5,x6,x7]

# Number 9
x0= bitarray('00000000')
x1= bitarray('00111100')
x2= bitarray('00100100')
x3= bitarray('00111100')
x4= bitarray('00000100')
x5= bitarray('00000100')
x6= bitarray('00111100')
x7= bitarray('00000000')
number_9=[x0,x1,x2,x3,x4,x5,x6,x7]

# Symbol -
x0= bitarray('00000000')
x1= bitarray('00000000')
x2= bitarray('00000000')
x3= bitarray('00111100')
x4= bitarray('00000000')
x5= bitarray('00000000')
x6= bitarray('00000000')
x7= bitarray('00000000')
symbol_hyph=[x0,x1,x2,x3,x4,x5,x6,x7]

# Symbol dot
x0= bitarray('00000000')
x1= bitarray('00000000')
x2= bitarray('00000000')
x3= bitarray('00000000')
x4= bitarray('00000000')
x5= bitarray('00001100')
x6= bitarray('00001100')
x7= bitarray('00000000')
symbol_dot=[x0,x1,x2,x3,x4,x5,x6,x7]

# Symbol at
x0= bitarray('00000000')
x1= bitarray('00111100')
x2= bitarray('00101100')
x3= bitarray('00101100')
x4= bitarray('00111100')
x5= bitarray('00100100')
x6= bitarray('00111000')
x7= bitarray('00000000')
symbol_at=[x0,x1,x2,x3,x4,x5,x6,x7]

# Symbol ?
x0= bitarray('00000000')
x1= bitarray('00111100')
x2= bitarray('00000100')
x3= bitarray('00111100')
x4= bitarray('00100000')
x5= bitarray('00100000')
x6= bitarray('00100000')
x7= bitarray('00000000')
symbol_qm=[x0,x1,x2,x3,x4,x5,x6,x7]

# Symbol !
x0= bitarray('00000000')
x1= bitarray('00001000')
x2= bitarray('00001000')
x3= bitarray('00001000')
x4= bitarray('00001000')
x5= bitarray('00000000')
x6= bitarray('00001000')
x7= bitarray('00000000')
symbol_em=[x0,x1,x2,x3,x4,x5,x6,x7]

# Symbol _
x0= bitarray('00000000')
x1= bitarray('00000000')
x2= bitarray('00000000')
x3= bitarray('00000000')
x4= bitarray('00000000')
x5= bitarray('00000000')
x6= bitarray('00111100')
x7= bitarray('00000000')
symbol_under=[x0,x1,x2,x3,x4,x5,x6,x7]

# Symbol hash
x0= bitarray('00000000')
x1= bitarray('00101000')
x2= bitarray('01111100')
x3= bitarray('00101000')
x4= bitarray('00101000')
x5= bitarray('01111100')
x6= bitarray('00101000')
x7= bitarray('00000000')
symbol_hash=[x0,x1,x2,x3,x4,x5,x6,x7]

# Symbol plus
x0= bitarray('00000000')
x1= bitarray('00000000')
x2= bitarray('00000000')
x3= bitarray('00001000')
x4= bitarray('00011100')
x5= bitarray('00001000')
x6= bitarray('00000000')
x7= bitarray('00000000')
symbol_plus=[x0,x1,x2,x3,x4,x5,x6,x7]

# Symbol equals
x0= bitarray('00000000')
x1= bitarray('00000000')
x2= bitarray('00000000')
x3= bitarray('00111100')
x4= bitarray('00000000')
x5= bitarray('00111100')
x6= bitarray('00000000')
x7= bitarray('00000000')
symbol_equals=[x0,x1,x2,x3,x4,x5,x6,x7]

# Symbol dollar
x0= bitarray('00000000')
x1= bitarray('00111100')
x2= bitarray('00100000')
x3= bitarray('00111100')
x4= bitarray('00010100')
x5= bitarray('00111100')
x6= bitarray('00010000')
x7= bitarray('00000000')
symbol_dollar=[x0,x1,x2,x3,x4,x5,x6,x7]

# Symbol ast
x0= bitarray('00000000')
x1= bitarray('00000000')
x2= bitarray('01010100')
x3= bitarray('00111000')
x4= bitarray('01111100')
x5= bitarray('00111000')
x6= bitarray('01010100')
x7= bitarray('00000000')
symbol_ast=[x0,x1,x2,x3,x4,x5,x6,x7]

# Symbol cb
x0= bitarray('00000000')
x1= bitarray('00010000')
x2= bitarray('00001000')
x3= bitarray('00000100')
x4= bitarray('00000100')
x5= bitarray('00001000')
x6= bitarray('00010000')
x7= bitarray('00000000')
symbol_cb=[x0,x1,x2,x3,x4,x5,x6,x7]

# Symbol ob
x0= bitarray('00000000')
x1= bitarray('00000100')
x2= bitarray('00001000')
x3= bitarray('00010000')
x4= bitarray('00010000')
x5= bitarray('00001000')
x6= bitarray('00000100')
x7= bitarray('00000000')
symbol_ob=[x0,x1,x2,x3,x4,x5,x6,x7]

# Symbol dbq
x0= bitarray('00000000')
x1= bitarray('00101000')
x2= bitarray('00101000')
x3= bitarray('00000000')
x4= bitarray('00000000')
x5= bitarray('00000000')
x6= bitarray('00000000')
x7= bitarray('00000000')
symbol_dbq=[x0,x1,x2,x3,x4,x5,x6,x7]


# Symbol gt
x0= bitarray('00000000')
x1= bitarray('00000000')
x2= bitarray('00110000')
x3= bitarray('00011000')
x4= bitarray('00000100')
x5= bitarray('00011000')
x6= bitarray('00110000')
x7= bitarray('00000000')
symbol_gt=[x0,x1,x2,x3,x4,x5,x6,x7]

# Symbol lt
x0= bitarray('00000000')
x1= bitarray('00000000')
x2= bitarray('00001100')
x3= bitarray('00011000')
x4= bitarray('00100000')
x5= bitarray('00011000')
x6= bitarray('00001100')
x7= bitarray('00000000')
symbol_lt=[x0,x1,x2,x3,x4,x5,x6,x7]

# Special hart
x0= bitarray('00000000')
x1= bitarray('01100110')
x2= bitarray('11111111')
x3= bitarray('11111111')
x4= bitarray('11111111')
x5= bitarray('01111110')
x6= bitarray('00111100')
x7= bitarray('00011000')
special_hart=[x0,x1,x2,x3,x4,x5,x6,x7]

# Special smilie
x0= bitarray('00111100')
x1= bitarray('01000010')
x2= bitarray('10100101')
x3= bitarray('10000001')
x4= bitarray('10100101')
x5= bitarray('10011001')
x6= bitarray('01000010')
x7= bitarray('00111100')
special_smilie=[x0,x1,x2,x3,x4,x5,x6,x7]

# Special degrees
x0= bitarray('00000000')
x1= bitarray('00011100')
x2= bitarray('00010100')
x3= bitarray('00011100')
x4= bitarray('00000000')
x5= bitarray('00000000')
x6= bitarray('00000000')
x7= bitarray('00000000')
special_degrees=[x0,x1,x2,x3,x4,x5,x6,x7]

'''The mapping dictionary is called to translate a character from the message to be displayed
into the relevant bitarray'''

mapping = {}
mapping['A'] = letter_A
mapping['B'] = letter_B
mapping['C'] = letter_C
mapping['D'] = letter_D
mapping['E'] = letter_E
mapping['F'] = letter_F
mapping['G'] = letter_G
mapping['H'] = letter_H
mapping['I'] = letter_I
mapping['J'] = letter_J
mapping['K'] = letter_K
mapping['L'] = letter_L
mapping['M'] = letter_M
mapping['N'] = letter_N
mapping['O'] = letter_O
mapping['P'] = letter_P
mapping['Q'] = letter_Q
mapping['R'] = letter_R
mapping['S'] = letter_S
mapping['T'] = letter_T
mapping['U'] = letter_U
mapping['V'] = letter_V
mapping['W'] = letter_W
mapping['X'] = letter_X
mapping['Y'] = letter_Y
mapping['Z'] = letter_Z
mapping['0'] = number_0
mapping['1'] = number_1
mapping['2'] = number_2
mapping['3'] = number_3
mapping['4'] = number_4
mapping['5'] = number_5
mapping['6'] = number_6
mapping['7'] = number_7
mapping['8'] = number_8
mapping['9'] = number_9
mapping['@'] = symbol_at
mapping['.'] = symbol_dot
mapping['-'] = symbol_hyph
mapping['?'] = symbol_qm
mapping['!'] = symbol_em
mapping[' '] = letter_space
mapping['_'] = symbol_under
mapping['#'] = symbol_hash
mapping['+'] = symbol_plus
mapping['='] = symbol_equals

mapping['$'] = symbol_dollar
mapping['"'] = symbol_dbq
mapping[')'] = symbol_cb
mapping['('] = symbol_ob
mapping['*'] = symbol_ast
mapping['>'] = symbol_gt
mapping['<'] = symbol_lt
mapping['heart'] = special_hart
mapping['smile'] = special_smilie
mapping['degrs'] = special_degrees


'''Characters are normally 4x6 high although some (e.g m, w) are wider and others are narrower 
(e.g. !, I, J). In order for these to be proiperly spaced in a word (i.e seperated by 2 blank columns,
any additional wide or narrow letters must be added to the appropriate list)'''

narrows = [symbol_plus,letter_I,letter_J,letter_T,symbol_em, symbol_ob, symbol_cb,symbol_plus]
super_narrow = [letter_space,special_degrees]
wides = [letter_M,letter_W,symbol_ast,symbol_hash,special_smilie, special_hart]
super_wides = [special_smilie, special_hart]
specials = ['~heart','~smile','~degrs']


#It assumes the Pi/hat will orientated with the long side of the Pi without any connectors on
#the bottom, i.e. the Hat will be rotated 90 degrees clockwise (assuming the "UNICORN HAT" label and 
#Pimoroni logo are normally at the bottom of the hat. If you want to use a different orientation  
#then you can alter the UH.rotation value in the show_letter function below. You may also need to adjust or omit
#the flip call which is used to ensure that the bitarray definitions in uhscroll_letters are the correct 
#way round for easy reading

flip = [7,6,5,4,3,2,1,0]


x0= bitarray('00000000')
x1= bitarray('00111100')
x2= bitarray('01000010')
x3= bitarray('01011010')
x4= bitarray('01011010')
x5= bitarray('01000010')
x6= bitarray('00111100')
x7= bitarray('00000000')

letterred = [x0,x1,x2,x3,x4,x5,x6,x7]

x0= bitarray('00000000')
x1= bitarray('00000000')
x2= bitarray('00111100')
x3= bitarray('00100100')
x4= bitarray('00100100')
x5= bitarray('00111100')
x6= bitarray('00000000')
x7= bitarray('00000000')

letterWhite = [x0,x1,x2,x3,x4,x5,x6,x7]

x0= bitarray('00000000')
x1= bitarray('00011000')
x2= bitarray('00111100')
x3= bitarray('00011000')
x4= bitarray('00011000')
x5= bitarray('00000000')
x6= bitarray('00000000')
x7= bitarray('00000000')

letterredS = [x0,x1,x2,x3,x4,x5,x6,x7]

x0= bitarray('00000000')
x1= bitarray('00100000')
x2= bitarray('01000000')
x3= bitarray('00100000')
x4= bitarray('00100000')
x5= bitarray('00011000')
x6= bitarray('00000000')
x7= bitarray('00000000')

letterWhiteS = [x0,x1,x2,x3,x4,x5,x6,x7]

#smiles
x0= bitarray('00000000')
x1= bitarray('00100100')
x2= bitarray('00100100')
x3= bitarray('00000000')
x4= bitarray('01000010')
x5= bitarray('00111100')
x6= bitarray('00000000')
x7= bitarray('00000000')

Smile0 = [x0,x1,x2,x3,x4,x5,x6,x7]

x0= bitarray('11100111')
x1= bitarray('00100100')
x2= bitarray('00100100')
x3= bitarray('00000000')
x4= bitarray('01000010')
x5= bitarray('00111100')
x6= bitarray('00000000')
x7= bitarray('00000000')

Smile1 = [x0,x1,x2,x3,x4,x5,x6,x7]

x0= bitarray('00000000')
x1= bitarray('11100111')
x2= bitarray('00100100')
x3= bitarray('00000000')
x4= bitarray('01000010')
x5= bitarray('00111100')
x6= bitarray('00000000')
x7= bitarray('00000000')

Smile2 = [x0,x1,x2,x3,x4,x5,x6,x7]

x0= bitarray('00000000')
x1= bitarray('00000000')
x2= bitarray('11100111')
x3= bitarray('00000000')
x4= bitarray('01000010')
x5= bitarray('00111100')
x6= bitarray('00000000')
x7= bitarray('00000000')

Smile3 = [x0,x1,x2,x3,x4,x5,x6,x7]

x0= bitarray('00000000')
x1= bitarray('00100100')
x2= bitarray('00100100')
x3= bitarray('00000000')
x4= bitarray('01000010')
x5= bitarray('00111100')
x6= bitarray('00000000')
x7= bitarray('00000000')

Smile1A = [x0,x1,x2,x3,x4,x5,x6,x7]

x0= bitarray('00100100')
x1= bitarray('01011010')
x2= bitarray('01011010')
x3= bitarray('00100100')
x4= bitarray('01000010')
x5= bitarray('00111100')
x6= bitarray('00000000')
x7= bitarray('00000000')

Smile2A = [x0,x1,x2,x3,x4,x5,x6,x7]

x0= bitarray('01011010')
x1= bitarray('10011001')
x2= bitarray('10011001')
x3= bitarray('01011010')
x4= bitarray('01100110')
x5= bitarray('00111100')
x6= bitarray('00000000')
x7= bitarray('00000000')

Smile3A = [x0,x1,x2,x3,x4,x5,x6,x7]

def ShowBuls(brightness, Status):
	UH.rotation(90)		
	for i in range(8):
		for j in range(8):
			if Status == "Amp":
				if letterred[j][i]:
					UH.set_pixel(j,flip[i],brightness,0,0)
				else:
					UH.set_pixel(j,flip[i],0,0,0)
			else:
				if letterredS[j][i]:
					UH.set_pixel(j,flip[i],brightness,0,0)
				else:
					UH.set_pixel(j,flip[i],0,0,0)

	for i in range(8):
		for j in range(8):
			if Status == "Amp":
				if letterWhite[j][i]:
					UH.set_pixel(j,flip[i],brightness,brightness,brightness)
			else:
				if letterWhiteS[j][i]:
					UH.set_pixel(j,flip[i],brightness,brightness,brightness)				

	UH.show()

def ShowSmileAuto(brightness, smilestage):
	UH.rotation(90)		
	if Smilestage < 61:
		print("1")
		UH.clear()
		for i in range(8):
			for j in range(8):
				if Smile1A[j][i]:
					UH.set_pixel(j,flip[i],brightness,0,0)
				else:
					UH.set_pixel(j,flip[i],0,0,0)
	elif Smilestage == 61:
		UH.clear()
		for i in range(8):
			for j in range(8):
				if Smile2A[j][i]:
					UH.set_pixel(j,flip[i],brightness,0,0)
				else:
					UH.set_pixel(j,flip[i],0,0,0)
	elif Smilestage == 62:
		UH.clear()
		for i in range(8):
			for j in range(8):
				if Smile3A[j][i]:
					UH.set_pixel(j,flip[i],brightness,0,0)
				else:
					UH.set_pixel(j,flip[i],0,0,0)
	elif Smilestage == 63:
		UH.clear()
		for i in range(8):
			for j in range(8):
				if Smile2A[j][i]:
					UH.set_pixel(j,flip[i],brightness,0,0)
				else:
					UH.set_pixel(j,flip[i],0,0,0)
	elif Smilestage == 64:
		UH.clear()
		for i in range(8):
			for j in range(8):
				if Smile1A[j][i]:
					UH.set_pixel(j,flip[i],brightness,0,0)
				else:
					UH.set_pixel(j,flip[i],0,0,0)
	UH.show()

def ShowSmile(brightness, Smilestage):
	UH.rotation(90)		
	if Smilestage < 61:
		print("1")
		UH.clear()
		for i in range(8):
			for j in range(8):
				if Smile0[j][i]:
					UH.set_pixel(j,flip[i],brightness,0,0)
				else:
					UH.set_pixel(j,flip[i],0,0,0)
	elif Smilestage == 61:
		UH.clear()
		for i in range(8):
			for j in range(8):
				if Smile1[j][i]:
					UH.set_pixel(j,flip[i],brightness,0,0)
				else:
					UH.set_pixel(j,flip[i],0,0,0)
	elif Smilestage == 62:
		UH.clear()
		for i in range(8):
			for j in range(8):
				if Smile2[j][i]:
					UH.set_pixel(j,flip[i],brightness,0,0)
				else:
					UH.set_pixel(j,flip[i],0,0,0)
	elif Smilestage == 63:
		UH.clear()
		for i in range(8):
			for j in range(8):
				if Smile3[j][i]:
					UH.set_pixel(j,flip[i],brightness,0,0)
				else:
					UH.set_pixel(j,flip[i],0,0,0)
	elif Smilestage == 64:
		UH.clear()
		for i in range(8):
			for j in range(8):
				if Smile2[j][i]:
					UH.set_pixel(j,flip[i],brightness,0,0)
				else:
					UH.set_pixel(j,flip[i],0,0,0)
	elif Smilestage == 65:
		UH.clear()
		for i in range(8):
			for j in range(8):
				if Smile1[j][i]:
					UH.set_pixel(j,flip[i],brightness,0,0)
				else:
					UH.set_pixel(j,flip[i],0,0,0)
	elif Smilestage == 66:
		UH.clear()
		for i in range(8):
			for j in range(8):
				if Smile0[j][i]:
					UH.set_pixel(j,flip[i],brightness,0,0)
				else:
					UH.set_pixel(j,flip[i],0,0,0)
	UH.show()


		


def show_letter(letter,colour,brightness): #displays a single letter on th UH
	UH.rotation(90)		
	for i in range(8):
		for j in range(8):
			if letter[j][i]:
				if colour == 'red':
					UH.set_pixel(j,flip[i],brightness,0,0)
				elif colour == 'green':
					UH.set_pixel(j,flip[i],0,brightness,0)
				elif colour == 'blue':
					UH.set_pixel(j,flip[i],0,0,brightness)
				elif colour == 'white':
					UH.set_pixel(j,flip[i],brightness,brightness,brightness)
				elif colour == 'pink':
					UH.set_pixel(j,flip[i],brightness,52,179)
				elif colour == 'cyan':
					UH.set_pixel(j,flip[i],0,brightness,brightness)
				elif colour == 'yellow':
					UH.set_pixel(j,flip[i],brightness,brightness,0)
				elif colour == 'orange':
					UH.set_pixel(j,flip[i],brightness,128,0)
			else:
				UH.set_pixel(j,flip[i],0,0,0)

	UH.show()

def scroll_letter(letter,colour,brightness,speed): # scrolls a single letter across the UH
	for i in range(8):
		for p in range(6):
			letter[i].insert(0,False)
	for s in range(14):
		show_letter(letter,colour,brightness)
		time.sleep(speed)
		for i in range(8):
			letter[i].pop(0)
			letter[i].append(0)

'''scrolling is achieved by redrawing the letter with a column of the bitarray shifted to the left and a new blank column
added to the right'''
def scroll_word(word,colour,brightness,speed): # scrolls a word across the UH
    for s in range(len(word[0])):
        show_letter(word,colour,brightness)
        time.sleep(speed)
        for i in range(8):
            word[i].pop(0)
            word[i].append(0)

def make_word(words): # takes a list of chars and concats into a word by making one big bitarray
	bigword = [bitarray(''),bitarray(''), bitarray(''),bitarray(''), bitarray(''),bitarray(''), bitarray(''),bitarray('')]
	for w in range(len(words)):
		for i in range(len(words[w])):
			bigword[i] = bigword[i] + words[w][i]
	return bigword
	
def trim_letter(letter): #trims a char's bitarray so that it can be joined without too big a gap
	trim = []
	for c in range(len(letter)):
		trim.append(letter[c].copy())
	if letter not in super_wides:
		for i in range(8):
			if letter not in wides:
				trim[i].pop(0)
			trim[i].pop(0)
			trim[i].pop(5)
			if letter in narrows:
				trim[i].pop(0)
			if letter in super_narrow:
				trim[i].pop(0)
				
	return trim

def map_character(chr):
	if chr in mapping:
		return mapping[chr]
	else:
		return mapping['_']

def load_message(message):
	unicorn_message = []
	message = '  ' + message # pad the message with a couple of spaces so it starts on the right
	skip = 0
	for ch in (range(len(message))):
		#print message[ch]
		if skip != 0:
			skip-=1
		else:
			if message[ch] == '~':
				spec = message[ch+1] + message[ch+2] + message[ch+3] + message[ch+4] + message[ch+5]
				unicorn_message.append(trim_letter(map_character(spec)))
				skip = 5
			else:
				unicorn_message.append(trim_letter(map_character(message[ch].upper())))
		
	return(unicorn_message)

def unicorn_scroll(text,colour,brightness,speed):
	#try:
	scroll_word(make_word(load_message(text)),colour,brightness,speed)
	#except: 
		#print 'Enter unicorn_scroll(message,colour,brightness,speed) where '
		#print 'message is a string, colour is either red,white,blue,green,pink, yellow, orange or cyan'
		#print 'brightness is a integer 0-255 and speed is the time between chars'

import random

def randcolor():
    colors = ['red', 'white', 'blue', 'green', 'pink', 'yellow', 'orange']
    Numcol = random.randrange(0, len(colors))
    color = colors[Numcol]
    return color

#funny_phrases = [
#    "404 Humor not found",
#    "Im not lazy im in energy-saving mode",
#    "When testing is over. you will be baked and there will be cake.",
#    "ChatGPT is my best friend",
#    "Not antisocial just user unfriendly",
#    "hold on justa little while longer - hold on justa little while longer",
#    "it works why",
#    "flip the world",
#    "lttstore.com",
#    "binary is as easy as 01 10 11",
#    "my attitude isnt bad its in beta",
#    "ctrl+c ctrl+v",
#    "i use arch btw",
#    "wpi bye",
#    "help i am blind",
#    "omg they killed kenny",
#    "a robot gets arrested - charged with battery",
#    "does r2d2 have any brothers - no only transitors",
#	"Before we start. however. keep in mind that while fun and learning are the primary goals of all enrichment center activitys. serious injuries may occur.",
#	"For your own safety and the safety of others please refrain from touching (bzzzzzt)",
#	"Lets be honest. neither one of us knows what that thing does. just put it in the corner. and Ill deal with it later.",
#	"How Are You Holding Up? Because Im A Potato.",
#	"Aperture Science",
#    "Before we start. however. keep in mind that while fun and learning are the primary goals of all enrichment center activitys. serious injuries may occur.",
#    "For your own safety and the safety of others please refrain from touching (bzzzzzt)",
#    "Lets be honest. neither one of us knows what that thing does. just put it in the corner. and Ill deal with it later.",
#    "call me GLADOS",
#	"tiss buta scratch",
#]



try:
    ntinst = ntcore.NetworkTableInstance.getDefault()
    table = ntinst.getTable("UnicornHat")
except:
	print("starting in no connection mode")

Brightmulti = 1
Smilestage = 0
smileauto = 0

while True:
    try:
        Shoot = table.getString("status","False")
        tagid = table.getString("TagID", "")
    except:
        print("rip")
    if (Shoot != "ShootingAmp"):
        if(Shoot != "ShootingSpeaker"):
            if tagid == "":
                if(Shoot != "AutoStart"):
                   ShowSmile(100, Smilestage)
                else:
                    ShowSmileAuto(100, smileauto)
                    smileauto += 1
                    if smileauto > 64:
                        smileauto = 0
            else:
                unicorn_scroll(str("Tag", tagid), 'red', 100, 0.01)
    time.sleep(0.2)
    Smilestage += 1
    if Smilestage > 66:
        Smilestage = 0
    if(Shoot == "ShootingAmp"):
        UH.clear()
        ShowBuls(50 * Brightmulti, "Amp")
        Brightmulti += 0.35
        if Brightmulti > 3.9:
            Brightmulti = 1
    elif(Shoot == "ShootingSpeaker"):
        UH.clear()
        ShowBuls(50 * Brightmulti, "Speaker")
        Brightmulti += 0.35
        if Brightmulti > 3.9:
            Brightmulti = 1
