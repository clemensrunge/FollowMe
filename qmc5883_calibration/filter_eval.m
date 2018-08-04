L=50;
bitrate=50;
tone1 = sin((0:1/50:L)*0.1*pi)
figure;
plot((0:1/50:L),tone1);
spectrum = fft(tone1);
    P2 = abs(spectrum/L);
    P1 = P2(1:L/2+1);
    P1(2:end-1) = 2*P1(2:end-1);
    f = bitrate*(0:(L/2))/L;
    figure;
    plot(f,P1) 
