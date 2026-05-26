#ifndef REED_SOLOMON_H
#define REED_SOLOMON_H

void RS_EncodeMsg(const uint8_t *msg, int msg_len, uint8_t *codeword, int nsym);

#endif // REED_SOLOMON_H