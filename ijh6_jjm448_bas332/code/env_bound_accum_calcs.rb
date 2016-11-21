ISR_FREQUENCY = 100000 # 100 kHz DDS sample frequency
INCREMENTS_TO_OVERFLOW = 4294967296 # 2^32 = value at which envelope maxes/mins out

# keyframe points in modal envelope transitions ("frequency" of envelope rise/fall)
envelope_keyframes =
[
	[1000, 500], # 1ms rise, 2ms fall
	[1000, 10], # 1ms rise, 100ms fall
	[1000, 1.25], # 1ms rise, 800ms fall
	[10, 1.5], # 100ms rise, 667ms fall
	[2, 2], # 400ms rise, 400ms fall
	[1.5, 10], # 667ms rise, 100ms fall
	[1.25, 1000], # 800ms rise, 1ms fall
	[10, 1000], # 100ms rise, 1ms fall
	[500, 1000], # 2ms rise, 1ms fall
]

# freq = 1 / ( ( INCREMENTS_TO_OVERFLOW / PHASE_INCREMENT ) * ( 1 / ISR_FREQUENCY ) )
# 1 / freq = ( INCREMENTS_TO_OVERFLOW * ( 1 / ISR_FREQUENCY ) ) / PHASE_INCREMENT
# PHASE_INCREMENT = INCREMENTS_TO_OVERFLOW * ( 1 / ISR_FREQUENCY ) * freq
# round it to the nearest int because we store these as ints

env_increments = envelope_keyframes.map {|x| x.map {|y| ( INCREMENTS_TO_OVERFLOW * (1.0/ISR_FREQUENCY) * y).round } }

# print envelope keyframes in C header format
puts "#ifndef ENV_KEYFRAMES"
puts "#define ENV_KEYFRAMES\n"
puts
puts "// ENVELOPE KEYFRAMES:"
puts "//    - 1ms rise, 2ms fall"
puts "//    - 1ms rise, 100ms fall"
puts "//    - 1ms rise, 800ms fall"
puts "//    - 100ms rise, 667ms fall"
puts "//    - 400ms rise, 400ms fall"
puts "//    - 667ms rise, 100ms fall"
puts "//    - 800ms rise, 1ms fall"
puts "//    - 100ms rise, 1ms fall"
puts "//    - 2ms rise, 1ms fall"
puts
puts "const unsigned int atk_incs[] = {"
env_increments.each_with_index {|x, idx| puts "    #{x[0]}#{(idx != 8) ? "," : ""}"}
puts "};"
puts
puts "const unsigned int decay_incs[] = {"
env_increments.each_with_index {|x, idx| puts "    #{x[1]}#{(idx != 8) ? "," : ""}"}
puts "};"
puts
puts "const unsigned int num_keyframes = 9;"
puts
puts "#endif"