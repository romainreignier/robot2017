#MCUDESTDIR=../firmware/common/SerialComm
ROSDESTDIR=../ros_ws/src/snd_serial/src
NANOPBDIR=../firmware/common/nanopb

all:
	protoc --cpp_out=$(ROSDESTDIR) CommMsgs.proto
#	protoc --plugin=protoc-gen-nanopb=$(NANOPBDIR)/generator/protoc-gen-nanopb --nanopb_out=$(MCUDESTDIR) CommMsgs.proto

clean:
	rm $(ROSDESTDIR)/CommMsgs.pb.h $(ROSDESTDIR)/CommMsgs.pb.cc
#	rm $(MCUDESTDIR)/CommMsgs.pb.h $(MCUDESTDIR)/CommMsgs.pb.c

