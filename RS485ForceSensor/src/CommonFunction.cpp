


namespace CommonFunction{
    void ShortToChar(unsigned short Short,unsigned char *byte){
        byte[0] = Short & 0xFF;
        byte[1] = (Short >> 8) & 0xFF;
    }
}