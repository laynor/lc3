module Utils
  extend self

  def stdin_empty?
    old_timeout = STDIN.read_timeout
    STDIN.read_timeout = 0

    begin
      pk = STDIN.raw &.peek
    rescue
      pk = Bytes.empty
    end

    STDIN.read_timeout = old_timeout
    return pk.empty?
  end

  def bitsplit(value, lengths)
    v = value
    lengths.reverse.map { |len| 
      mask = 2 ** len - 1
      res  = v & mask
      v    = v >> len
      res
    }.reverse
  end

  def bit(word, i)
    (word >> i) & 1
  end 

  def bit?(word, i)
    bit(word, i) == 1
  end

  def sign_extend(x, bit_count)
    if bit?(x, bit_count - 1)
      x | (0xFFFF << bit_count)
    else  
      x
    end 
  end
end

