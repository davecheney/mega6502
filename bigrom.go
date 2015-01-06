package main

import "fmt"
import "io/ioutil"

func check(err error) {
	if err != nil {
		panic(err)
	}
}

var banks [16][]byte

func main() {
	var err error
	banks[0xa], err = ioutil.ReadFile("rom/A1A90.bin")
	check(err)
	rom, err := ioutil.ReadFile("rom/6502.rom.bin")
	check(err)
	banks[0xe] = rom[:0x1000]
	banks[0xf] = rom[0x1000:]
	fmt.Print("const uint8_t bigrom[] PROGMEM = { ")
	for _, b := range banks[0x0e:] {
		for _, c := range b {
			fmt.Printf("0x%0x, ", c)
		}
	fmt.Println()
	}
	fmt.Println("};")
}
