ISR_FREQUENCY = 100000 # 100 kHz DDS sample frequency
INCREMENTS_TO_OVERFLOW = 4294967296 # 2^32 = value at which phase accum overflows

c1 = 32.7032 # frequency (in hertz) of C1
twelfth_root_of_two = 2 ** (1.0/12) # for semitone intervals
freqs_to_c5 = [*0..48].map {|x| (c1 * (twelfth_root_of_two ** x))} # frequencies from C1 to C5

# freq = 1 / ( ( INCREMENTS_TO_OVERFLOW / PHASE_INCREMENT ) * ( 1 / ISR_FREQUENCY ) )
# 1 / freq = ( INCREMENTS_TO_OVERFLOW * ( 1 / ISR_FREQUENCY ) ) / PHASE_INCREMENT
# PHASE_INCREMENT = INCREMENTS_TO_OVERFLOW * ( 1 / ISR_FREQUENCY ) * freq
# round it to the nearest int because we store these as ints

phase_increments = freqs_to_c5.map {|x| ( INCREMENTS_TO_OVERFLOW * (1.0/ISR_FREQUENCY) * x).round }

# print phase increments
phase_increments.each {|x| puts x}