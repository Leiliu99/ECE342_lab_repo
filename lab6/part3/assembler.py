#!/usr/bin/env python3

import argparse
import os
import sys
from sys import exit
import subprocess

# Instruction dictionary. Used to look up instruction types and opcodes.
# [opcode, funct3, funct7, instr_type]
OPCODE = 0
FUNCT3 = 1
FUNCT7 = 2
ITYPE  = 3 # instr type
OP = {
	"lui":   ["0110111", None, None, "U"],
	"auipc": ["0010111", None, None, "U"],
	"jal":   ["1101111", None, None, "J"],
	"jalr":  ["1100111", "000", None, "I"],
	# branches
	"beq":   ["1100011", "000", None, "B"],
	"bne":   ["1100011", "001", None, "B"],
	"blt":   ["1100011", "100", None, "B"],
	"bge":   ["1100011", "101", None, "B"],
	"bltu":  ["1100011", "110", None, "B"],
	"bgeu":  ["1100011", "111", None, "B"],
	# loads
	"lb":    ["0000011", "000", None, "I"],
	"lh":    ["0000011", "001", None, "I"],
	"lw":    ["0000011", "010", None, "I"],
	"lbu":   ["0000011", "100", None, "I"],
	"lhu":   ["0000011", "101", None, "I"],
	# stores
	"sb":    ["0100011", "000", None, "S"],
	"sh":    ["0100011", "001", None, "S"],
	"sw":    ["0100011", "010", None, "S"],
	# ALU immediate
	"addi":  ["0010011", "000", None, "I"],
	"slli":  ["0010011", "001", "0000000", "I"],
	"slti":  ["0010011", "010", None, "I"],
	"sltiu": ["0010011", "011", None, "I"],
	"xori":  ["0010011", "100", None, "I"],
	"srli":  ["0010011", "101", "0000000", "I"],
	"srai":  ["0010011", "101", "0100000", "I"],
	"ori":   ["0010011", "110", None, "I"],
	"andi":  ["0010011", "111", None, "I"],
	# ALU standard
	"add":   ["0110011", "000", "0000000", "R"],
	"sub":   ["0110011", "000", "0100000", "R"],
	"sll":   ["0110011", "001", "0000000", "R"],
	"slt":   ["0110011", "010", "0000000", "R"],
	"sltu":  ["0110011", "011", "0000000", "R"],
	"xor":   ["0110011", "100", "0000000", "R"],
	"srl":   ["0110011", "101", "0000000", "R"],
	"sra":   ["0110011", "101", "0100000", "R"],
	"or":    ["0110011", "110", "0000000", "R"],
	"and":   ["0110011", "111", "0000000", "R"],
	# special instrs - Ignore
	# "fence"
	# "ecall"
	# "ebreak"
	# psudo instruction - this is the only one we have implemented
	"nop":   ["0010011", "000", None, "I"]
}

# Register dictionary. Translate from register name to reg number.
# https://en.wikichip.org/wiki/risc-v/registers
REG = {
	"zero": 0, "ra": 1, "sp": 2, "gp": 3, "tp": 4, 
	"t0": 5, "t1": 6, "t2": 7,
	"fp": 8, "s0": 8, "s1": 9 # yes these should both be 8
}
for i in range(0,8):
	REG[f"a{i}"] = 10 + i
for i in range(2,12):
	REG[f"s{i}"] = 16 + i
for i in range(3,7):
	REG[f"t{i}"] = 25 + i
for i in range(32):
	REG[f"x{i}"] = i


def removeExtension(path):
	"""Remove the file extenstion from a path/filename."""
	out_dir = os.path.dirname(path)
	out_name = os.path.basename(path).split('.')[0]
	return os.path.join(out_dir, out_name)	

def leftZeroPad(string, size):
	"""Pad the left of a number string with zeros and return a string with 'size' digits."""
	if len(string) > size:
		return string[-size:]
	if len(string) < size:
		string = '0' * (size - len(string)) + string
	return string[0:size]

def tohex(num, size):
	"""Convert the number to a hex string and sign extend to 'size' digits."""
	if num >= 0:
		string = hex(num)[2:]
		return leftZeroPad(string, size)
	# num < 0
	string = hex(2**(size*4)+num)[2:]
	return string[-size:]

def tobin(num, size):
	"""Convert the number to a binary string and sign extend to 'size' digits."""
	if num >= 0:
		string = bin(num)[2:]
		return leftZeroPad(string, size)
	# num < 0
	string = bin(2**(size)+num)[2:]
	return string[-size:]

def regStr(reg=0):
	"""Convert a register name into a register number."""
	if type(reg) == str:
		reg = REG.get(reg, reg)
		if type(reg) == str:
			return reg
	rs = bin(reg)[2:]
	return leftZeroPad(rs, 5)

def parseChars(num):
	"""Convert a string (enclosed in quotes) into a list of utf-8 numbers."""
	if num[0] == '"' and num[-1] == '"' or num[0] == "'" and num[-1] == "'":
		num = num[1:-1]
	charlist = []
	for c in num:
		charlist.append(ord(c))
	return charlist

def parseNum(num):
	"""Convert a number string or character string into number(s)."""
	if isinstance(num, int):
		return num
	if num[0] in ['"', "'"]:
		return parseChars(num)
	if num[0] == '-':
		return -1 * parseNum(num[1:])
	if len(num) >= 3 and num[0:2] == "0x":
		num = int(num, 16)
	elif len(num) >= 3 and num[0:2] == "0b":
		num = int(num, 2)
	else:
		num = int(num)
	return num

class Immediate:
	"""This is a class to assist with bit splicing the immediate value."""
	maxbits = 32
	def __init__(self, i):
		self.val = i
		self.str = None
	def toStr(self):
		self.str = tobin(self.val, self.maxbits)
		return self.str
	def get(self, high=maxbits-1, low=0):
		"""Get inclusive range of bits."""
		if not self.str:
			self.toStr()
		return self.str[self.maxbits-high-1:self.maxbits-low]
	def at(self, i):
		"""Get single bit."""
		return self.get(i, i)

def assemble(op="nop", rd=0, rs1=0, rs2=0, imm=0):
	"""Build a hex string representing an instruction."""
	if isinstance(imm, str):
		imm = parseNum(imm)
	if op == "nop":
		rd = 0
		rs1 = 0
		imm = 0
	O = OP[op]
	itype = O[ITYPE]
	imm = Immediate(imm)
	if itype == "R":
		instr = [O[FUNCT7], regStr(rs2), regStr(rs1), O[FUNCT3],
		regStr(rd), O[OPCODE]]
	elif itype == "I":
		if O[FUNCT7]:
			instr = [O[FUNCT7], imm.get(4, 0), regStr(rs1), O[FUNCT3], regStr(rd), O[OPCODE]]	
		else:
			instr = [imm.get(11, 0), regStr(rs1), O[FUNCT3], regStr(rd), O[OPCODE]]
	elif itype == "S":
		instr = [imm.get(11, 5), regStr(rs2), regStr(rs1), O[FUNCT3], imm.get(4, 0), O[OPCODE]]
	elif itype == "B":
		instr = [imm.at(12), imm.get(10, 5), regStr(rs2),
			regStr(rs1), O[FUNCT3], imm.get(4, 1), imm.at(11), O[OPCODE]]
	elif itype == "U":
		instr = [imm.get(31, 12), regStr(rd), O[OPCODE]]
	elif itype == "J":
		instr = [imm.at(20), imm.get(10, 1), imm.at(11), imm.get(19, 12), regStr(rd), O[OPCODE]]
	else:
		print(f"ERROR: Unknown instruction type '{itype}'.")
		print("Exiting.")
		exit(1)
	
	instr_str = "".join(instr)
	instr_str = hex(int(instr_str, 2))[2:]
	instr_str = leftZeroPad(instr_str, 8)
	return instr_str

def splitBytesInstr(instr):
	"""Rearrange bytes for endienness of hex file."""
	return f"{instr[6:8]} {instr[4:6]} {instr[2:4]} {instr[0:2]} "

def parseTag(imm, cur_addr, tags):
	"""Check if the immediate was given as a tag.
	If it was a tag returns a relative offset of the tag from the current address
	(as is desired for branches).
	If the true tag value (not relative) is desired (as for addi, etc.) set cur_addr = 0.
	"""
	tag_addr = tags.get(imm)
	if tag_addr != None:
		return tag_addr - cur_addr
	return parseNum(imm)

def parseInstr(line, line_num, line_addr, tags):
	"""Parse an instruction string into fields.
	Match the required fields (based on opcode) and assemble the instruction.
	"""
	try:
		if line[0] == "nop":
			comments = " ".join(line[1:])
			line = ["nop", 0, 0, 0, comments]
		O = OP.get(line[0])
		if not O:
			print(f"ERROR: Line {line_num+1}. Could not find opcode '{line[0]}' from line '{line}'.")
			print("Exiting.")
			exit(1)
	
		itype = O[ITYPE]
		N = 4
		if itype == "R":
			instr_str = assemble(line[0], rd=line[1], rs1=line[2], rs2=line[3])
		elif itype == "I":
			instr_str = assemble(line[0], rd=line[1], rs1=line[2],
			imm=parseTag(line[3], 0, tags))
		elif itype == "S" or itype == "B":
			instr_str = assemble(line[0], rs1=line[1], rs2=line[2],
			imm=parseTag(line[3], line_addr, tags))
		elif itype == "U":
			val = parseTag(line[2], 0, tags)
			# if tag, use top bits of tag
			if not tags.get(line[2]):				
				# if not tag, interpret at top 20 bits
				val = val << 12
			instr_str = assemble(line[0], rd=line[1], imm=val)
			N = 3
		elif itype == "J":
			instr_str = assemble(line[0], rd=line[1], imm=parseTag(line[2], line_addr, tags))
			N = 3
		else:
			print(f"ERROR: Line {line_num+1}. Unknown instruction type '{itype}'.")
			print("Exiting.")
			exit(1)
		if len(line) > N:
			comments = " ".join(line[N:])
		return [instr_str, comments]
	except IndexError:
		print(f"ERROR: Line {line_num+1}. Not enough operands in '{line}'.")
		print("Exiting.")
		exit(1)
	except ValueError:
		print(f"ERROR: Line {line_num+1}. Value error in '{line}'.")
		print("Exiting.")
		exit(1)
	
def parseLine(line):
	"""Split a line into a list of pieces.
	If line only has comments: ["", comment_string]
	If line is a tag: [tag_string, comment_string]
	If line is an instructon: [operation, regs, imm, comment]
	"""
	# remove comments
	end1 = line.find("#")
	end2 = line.find("//")
	if end1 != -1 and (end2 == -1 or end1 < end2):
		comment = line[end1:]
		line = line[:end1]
	elif end2 != -1:
		comment = line[end2:]
		line = line[:end2]
	else:
		comment = ""
	line = line.strip()
	if not line: # line has no instr, only comments
		return ["", comment]
	tmp_line = line.split()
	line = []
	for segment in tmp_line:
		for s in segment.split(','):
			if s: line.append(s)
	# line[0] must either be an opcode or tag
	if line[0][-1] == ":": # tag
		return [line[0], comment]
	line.append(comment)
	return line

def assembleLines(lines, add_comments=True):
	"""Assemble a list of lines."""

	# split each line if not already split
	for i in range(len(lines)):
		if isinstance(lines[i], str):
			lines[i] = parseLine(lines[i])
	
	data = []
	instrs = []
	tags = {}
	addr = 0
	i = 0
	while i < len(lines):
		if not lines[i][0]: # line with only comments
			i += 1
			continue
		
		if lines[i][0][0] == ".":
			if lines[i][0] == ".org":
				addr = parseNum(lines[i][1])
				lines.insert(i+1, ["@" + tohex(addr, 8), ''])
				i += 1
			elif lines[i][0] == ".dw":
				count = 0
				for num in reversed(lines[i][1:-1]):
					if num:
						data.append(i + count + 1)
						lines.insert(i+1, [num, ""])
						count += 1
				i += count
				addr += (4 * count)
			elif lines[i][0] == ".db":
				count = 0
				byte_list = lines[i][1:-1]
				while len(lines) > i+1 and (not lines[i+1][0] or lines[i+1][0] == ".db"):
					j = i+2
					next_is_byte = lines[i+1][0]
					while len(lines) > j and not next_is_byte:
						if lines[j][0] and lines[j][0] == ".db":
							next_is_byte = True
						j += 1
					if not next_is_byte:
						break
					i += 1
					byte_list.extend(lines[i][1:-1])
				byte_arr = []
				for b in byte_list:
					if b: byte_arr.append(b)
				while len(byte_arr) % 4 != 0:
					byte_arr.append(0)
				j = 0
				while j < len(byte_arr):
					byte_buffer = []
					for k in range(j, j+4):
						byte_buffer.append(tohex(parseNum(byte_arr[k]), 2))
					byte_buffer = " ".join(byte_buffer) + " "
					lines.insert(i+1, [byte_buffer, ""])
					count += 1
					j += 4
				i += count
				addr += (4 * count)
		elif lines[i][0] == ".string":
			# TODO implement .string
			print("ERROR: haven't yet implemented .string.")
			print("Exiting.")
			exit(1)
		elif lines[i][0][-1] == ":": # tag
			tags[lines[i][0][0:-1]] = addr
			lines[i][0] = "// " + lines[i][0]
		else: # instr
			instrs.append([i, addr, lines[i]])
			addr += 4
		i += 1

	# Convert .dw data into numbers
	# delayed until here so that all tags are available
	for index in data:
		num = lines[index][0]
		n = parseTag(num, 0, tags)
		n = tohex(n, 8)
		lines[index][0] = splitBytesInstr(n)
	
	# Assemble each instruction
	# (add address and original instruction as comments)
	# delayed until here so that all tags are available
	for i in range(len(instrs)):
		instr_line = instrs[i][0]
		instr_addr = instrs[i][1]
		line = parseInstr(instrs[i][2], instr_line, instr_addr, tags)
		line[0] = splitBytesInstr(line[0])
		if add_comments:
			instr_ = lines[instr_line]
			addr_str = tohex(instr_addr, 8)
			comment = "// 0x" + addr_str + ": " + instr_[0]+" "+", ".join(instr_[1:-1])
			if instr_[-1]:
				comment = comment + "\t" + instr_[-1]
			line = [line[0], comment]
		lines[instr_line] = line
	
	# Finalize formatting of lines and comments. Merge them into one string per line.
	final_lines = []
	if lines[0][0] != ".org":
		final_lines.append("@00000000")
	for i in range(len(lines)):
		# comment out things that are not hex data
		if lines[i][1] and lines[i][0] not in [".org", ".dw", ".db"]:
			if lines[i][1][0] == '#':
				lines[i][1] = "//" + lines[i][1][1:]
			elif len(lines[i][1]) < 2 or lines[i][1][0:2] != "//":
				lines[i][1] = "// " + lines[i][1]
		
		if not lines[i][-1]: # remove empty comments
			lines[i] = lines[i][:-1]
		
		if lines[i][0] == ".org": # comment out .org
			lines[i][0] = "// " + lines[i][0]

		if not lines[i][0]: # line with only comments
			if len(lines[i]) < 2:
				final_lines.append("") # empty line
			else:
				final_lines.append(lines[i][1])
		elif lines[i][0] in [".dw", ".db"]:
			if lines[i][-1] and len(lines[i][-1]) >= 2 and (lines[i][-1][0] == '#' or lines[i][-1][0:2] == "//"):
				# last item is a comment
				line = "// " + lines[i][0] + " " + ", ".join(lines[i][1:-1]) + "\t" + lines[i][-1]
			else:
				line = "// " + lines[i][0] + " " + ", ".join(lines[i][1:])
			final_lines.append(line)
		else:
			final_lines.append("\t".join(lines[i]))			
	
	return final_lines


def main():
	prologue = "Assemble a file line for line into RISC-V assembly."
	epilogue="""This is a very simple assembler. It translates line by line and only tags can cross lines. Tags must be on separate lines from instructions. Comments are prefixed by '#' or '//' (no block comments). Following is an example program.

main:
	addi t0, zero, 0 // loop counter
	addi a0, zero, 0b1111 // loop max
	addi s2, zero, 0xff00 // addr
loop:
	bge t0, a0, end // if done
	lw t2, s2, 0x0 // lw dest, addr, offset
	andi t2, t2, 0xFF
	sw s2, t2, 0x0 // sw addr, data, offset
	addi t0, t0, 4
	jal, zero, loop // jump to top of loop
end:
"""

	parser = argparse.ArgumentParser(description=prologue, epilog=epilogue, formatter_class=argparse.RawTextHelpFormatter)

	parser.add_argument("input_file", help="File to be compiled.")
	parser.add_argument("-o", "--output-file", metavar="PATH", default="", help="Hex output file name.")
	parser.add_argument("-x", "--no-comments", action='store_true', default=False, help="Do not include source line comments.")

	parser.add_argument("-L", metavar="N", type=int, default=7, help="Lab number (5 or 7).")
	
	args = parser.parse_args()

	lab_num = f"L{args.L}"

	input_file = args.input_file
	if args.output_file:
		hex_file = args.output_file
	else:
		hex_file = removeExtension(input_file) + f"-{lab_num}" + ".hex"
	
	defines = [lab_num]
	lines = []
	ifdef = []
	with open(input_file, 'r+') as f:
		for line in f:
			if line.startswith("#define"):
				defines.append(line.split("#define")[1].split()[0].strip())
			elif line.startswith("#ifdef"):
				ifdef.append([line.split("#ifdef")[1].split()[0].strip(), False])
				ifdef[-1][1] = ifdef[-1][0] in defines
			elif line.startswith("#elif"):
				ifdef.pop()
				ifdef.append([line.split("#ifdef")[1].split()[0].strip(), False])
				ifdef[-1][1] = ifdef[-1][0] in defines
			elif line.startswith("#else"):
				ifdef[-1][1] = not ifdef[-1][1]
			elif line.startswith("#endif"):
				ifdef.pop()
			elif not ifdef or ifdef[-1][1]:
				lines.append(line[:-1]) # remove newline char
	
	if ifdef:
		print("ERROR: unbalanced #ifdef")
		print("Exiting.")
		exit(1)
		
	lines = assembleLines(lines, not args.no_comments)

	with open(hex_file, 'w+') as f:
		for line in lines:
			f.write(line + "\n")

if __name__ == "__main__":
    main()
