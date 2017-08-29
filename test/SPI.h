typedef enum { LSBFIRST = 0, MSBFIRST = 1 } SPIEndian;
typedef enum { SPI_MODE0 = 0, } SPIMode;

struct SPISettings {
	int		rate;
	SPIEndian	endian;
	SPIMode		mode;
	SPISettings(int _rate, int _mode, int _endian) : rate(_rate), mode((SPIMode)_mode), endian((SPIEndian)_endian) {}
};

struct SPI {
	void	begin() {}
	uint8_t	transfer(uint8_t out) { return out; }
	void	beginTransaction(SPISettings s) {}
	void	endTransaction() {}
} SPI;
