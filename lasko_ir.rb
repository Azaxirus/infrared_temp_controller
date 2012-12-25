require 'csv'

f = CSV.read(ARGV[0])
previous_time = f[2][0].to_f
blips = 0
space = 1500

list = []

f[3..-1].each do |s|
  time = s[0].to_f
  delta = ((time - previous_time) * 100000000).to_i
  previous_time = time
  if s[1].to_i == 1
    if delta < space
      blips = blips + 1
    else
      if blips > 0
        list << blips / 2
        blips = 0
      end
      mylong = (delta / 100).to_i
      list << mylong
    end
  else
    if delta < space
      blips = blips + 1
    else
      if blips > 0
        list << blips / 2
        blips = 0
      end
      mylong = (delta / 100).to_i
      list << mylong
    end
  end
end

p list
  
