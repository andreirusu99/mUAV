CPPFLAGS+= -I/opt/vc/include/ -I/opt/vc/include/interface/vcos/pthreads -I/opt/vc/include/interface/vmcs_host/linux
LDFLAGS+= -lfreetype -lz
LDFLAGS+=-L/opt/vc/lib/ -lGLESv2 -lEGL -lopenmaxil -lbcm_host -lvcos -lvchiq_arm -lpthread -lrt -lm -lshapes

all: osd

/tmp/%.o: %.c
	gcc -c -o $@ $< $(CPPFLAGS)
 

osd: /tmp/main.o /tmp/frsky.o /tmp/render.o /tmp/telemetry.o /tmp/ltm.o /tmp/mavlink.o
	gcc -o /tmp/$@ $^ $(LDFLAGS)


clean:
	rm -f /tmp/osd /tmp/*.o /tmp/*~

