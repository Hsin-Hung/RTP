#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/file.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <queue>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>

/* ******************************************************************
   ARQ NETWORK EMULATOR: VERSION 1.1  J.F.Kurose
   Modified by Chong Wang on Oct.21,2005 for csa2,csa3 environments

   This code should be used for PA2, unidirectional data transfer protocols
   (from A to B)
   Network properties:
   - one way network delay averages five time units (longer if there
     are other messages in the channel for Pipelined ARQ), but can be larger
   - packets can be corrupted (either the header or the data portion)
     or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
     (although some can be lost).
**********************************************************************/

/* a "msg" is the data unit passed from layer 5 (teachers code) to layer  */
/* 4 (students' code).  It contains the data (characters) to be delivered */
/* to layer 5 via the students transport level protocol entities.         */
struct msg
{
  char data[20];
};

/* a packet is the data unit passed from layer 4 (students code) to layer */
/* 3 (teachers code).  Note the pre-defined packet structure, which all   */
/* students must follow. */
struct pkt
{
  int seqnum;
  int acknum;
  int checksum;
  int sack[5];
  char payload[20];
};

/*- Your Definitions
  ---------------------------------------------------------------------------*/

void A_output(msg message);
#define BUFFER_SIZE 5000
#define TIMER_INCR 1.0
double cur_time = 0.0;
struct packet_slot
{
  struct pkt packet;
  double timeout;
  /*
    key:
    sender: 1 -> sent not ack, 2 -> acked
    receiver: 1 -> acked, 2 -> delivered
  */
  int key;
};

std::vector<struct packet_slot> send_window(BUFFER_SIZE);
std::vector<struct packet_slot> rcv_window(BUFFER_SIZE);

std::queue<struct msg> buffer; /* msg buffer for when send window is full */

struct Sender
{
  int next_seq;
  int send_base;
  std::vector<int> latest_sacks;

} Sender_A;

struct Receiver
{
  int rcv_base;
  int expected_seq;
  int last_ack;
  std::vector<int> latest_sacks;
  int next_sack;

} Receiver_B;

double A_from_layer5 = 0.0;
double A_to_B = 0.0;
double B_from_A = 0.0;
double A_retrans_B = 0.0;
double B_to_layer5 = 0.0;
double B_acks = 0.0;
double corrupt = 0.0;
double loss_ratio = 0.0;
double corrupt_ratio = 0.0;
std::vector<struct time_stats> rtt(BUFFER_SIZE), comm(BUFFER_SIZE);
double totalRtt = 0.0, totalComm = 0.0;
double totalRttPackets = 0.0, totalCommPackets = 0.0;

/* Please use the following values in your program */

#define A 0
#define B 1
#define FIRST_SEQNO 0

/*- Declarations ------------------------------------------------------------*/
void restart_rxmt_timer(void);
void tolayer3(int AorB, struct pkt packet);
void tolayer5(char datasent[20]);

void starttimer(int AorB, double increment);
void stoptimer(int AorB);

/* WINDOW_SIZE, RXMT_TIMEOUT, and TRACE are inputs to the program;
   Please set an appropriate value for LIMIT_SEQNO.
   You have to use these variables in your
   routines --------------------------------------------------------------*/

extern int WINDOW_SIZE;     // size of the window
extern int LIMIT_SEQNO;     // when sequence number reaches this value,                                     // it wraps around
extern double RXMT_TIMEOUT; // retransmission timeout
extern int TRACE;           // trace Level, for your debug purpose
extern double time_now;     // simulation time, for your debug purpose

/********* YOU MAY ADD SOME ROUTINES HERE ********/

/* add an Acked packet to SACK */
void add_sack(int seq)
{

  if (Receiver_B.latest_sacks.size() < 5)
  {
    Receiver_B.latest_sacks.push_back(seq);
  }
  else
  {
    Receiver_B.latest_sacks.at(Receiver_B.next_sack) = seq;
  }

  Receiver_B.next_sack = (Receiver_B.next_sack + 1) % 5;
}

/* computer checksum that simply adds up all fields in the given packet */
int compute_checksum(struct pkt *packet)
{

  int checksum = 0;
  checksum += packet->seqnum;
  checksum += packet->acknum;

  for (int i = 0; i < 5; i++)
  {
    checksum += packet->sack[i];
  }
  for (int i = 0; i < 20; i++)
  {
    checksum += packet->payload[i];
  }
  return checksum;
}

/* check if the given sequence number is within the window */
int is_within_window(int base, int seq)
{

  return (seq >= base && seq < base + WINDOW_SIZE) || (seq < base && seq + LIMIT_SEQNO < base + WINDOW_SIZE);
}

int is_send_window_full()
{

  return Sender_A.next_seq == (Sender_A.send_base + WINDOW_SIZE) % LIMIT_SEQNO;
}

void send_ack(int acknum)
{
  struct pkt packet;
  for (int i = 0; i < 5; ++i)
  {
    if (i < Receiver_B.latest_sacks.size())
    {
      packet.sack[i] = Receiver_B.latest_sacks.at(i);
    }
    else
    {
      packet.sack[i] = -1;
    }
  }

  packet.acknum = acknum;
  packet.checksum = compute_checksum(&packet);
  tolayer3(B, packet);
}

/********* STUDENTS WRITE THE NEXT SIX ROUTINES *********/

/* called from layer 5, passed the data to be sent to other side */
void A_output(msg message)
{

  ++A_from_layer5;

  if (is_send_window_full())
  {
    buffer.push(message);
    return;
  }

  /* get a new packet slot for the new packet */
  struct packet_slot *p = &send_window.at(Sender_A.next_seq);

  /* set up packet */
  memcpy(p->packet.payload, message.data, sizeof(message.data));
  p->packet.acknum = 0;
  p->packet.seqnum = Sender_A.next_seq;
  p->packet.checksum = compute_checksum(&(p->packet));
  p->key = 1; /* packet is sent but not acked */
  p->timeout = (time_now + RXMT_TIMEOUT);

  tolayer3(A, p->packet);
  if (Sender_A.send_base == Sender_A.next_seq)
  {
    starttimer(A, RXMT_TIMEOUT);
  }
  Sender_A.next_seq = (Sender_A.next_seq + 1) % LIMIT_SEQNO;
  ++A_to_B;
}

/* called from layer 3, when a packet arrives for layer 4 */
void A_input(pkt packet)
{

  if (packet.checksum != compute_checksum(&packet))
  {
    ++corrupt;
    std::cout << "Packet corrupted!\n"
              << std::endl;
    return;
  }

  if (!is_within_window(Sender_A.send_base, packet.acknum))
  {
    std::cout << "Not in window!\n"
              << std::endl;

    return;
  }

  /* slide the window after packets are Acked */
  int count = 0;
  if (packet.acknum < Sender_A.send_base)
  {

    count = packet.acknum - Sender_A.send_base + LIMIT_SEQNO;
  }
  else
  {

    count = packet.acknum - Sender_A.send_base;
  }

  Sender_A.send_base = (packet.acknum + 1) % LIMIT_SEQNO;

  /* send more packets after sliding window */
  while (count > 0)
  {
    if (buffer.empty())
      break;
    A_output(buffer.front());
    buffer.pop();
    --count;
  }
  stoptimer(A);
  if (Sender_A.send_base != Sender_A.next_seq)
  {
    starttimer(A, RXMT_TIMEOUT);
  }
}

/* called when A's timer goes off */
void A_timerinterrupt()
{
  int i = Sender_A.send_base;

  while (i != Sender_A.next_seq)
  {

    struct packet_slot *p = &send_window.at(i);
    /* only retransmit unAcked packets */
    if (std::find(Sender_A.latest_sacks.begin(), Sender_A.latest_sacks.end(), i) == Sender_A.latest_sacks.end())
    {
      tolayer3(A, p->packet);
      ++A_retrans_B;
    }

    i = (i + 1) % LIMIT_SEQNO;
  }

  starttimer(A, RXMT_TIMEOUT);
}

/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  Sender_A.next_seq = FIRST_SEQNO;
  Sender_A.send_base = FIRST_SEQNO;
  send_window.resize(LIMIT_SEQNO);
  for (int i = 0; i < BUFFER_SIZE; i++)
  {
    send_window[i].key = 0;
  }
}

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(pkt packet)
{

  ++B_from_A;
  if (packet.checksum != compute_checksum(&packet))
  {
    std::cout << "packet corrupted!\n"
              << std::endl;
    ++corrupt;
    return;
  }

  /* update the SACK with 5 lastest Acked packets */
  if (is_within_window(Receiver_B.rcv_base, packet.seqnum))
  {
    add_sack(packet.seqnum);
  }

  /* only Ack packets in order */
  if (packet.seqnum != Receiver_B.expected_seq)
  {
    send_ack(Receiver_B.last_ack);
    ++B_acks;
    return;
  }

  struct packet_slot *p = &rcv_window.at(packet.seqnum);
  p->key = 0;
  tolayer5(packet.payload);
  ++B_to_layer5;
  /* add the packet into receiver window */
  memcpy(p->packet.payload, packet.payload, sizeof(packet.payload));
  p->packet.checksum = packet.checksum;
  p->packet.seqnum = packet.seqnum;
  p->packet.acknum = packet.acknum;
  send_ack(Receiver_B.expected_seq);
  ++B_acks;
  Receiver_B.last_ack = Receiver_B.expected_seq;
  Receiver_B.expected_seq = (Receiver_B.expected_seq + 1) % LIMIT_SEQNO;
}

/* the following rouytine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init()
{
  Receiver_B.expected_seq = FIRST_SEQNO;
  Receiver_B.last_ack = -1;
  rcv_window.resize(LIMIT_SEQNO);
}

/* called at end of simulation to print final statistics */
void Simulation_done()
{
  /* TO PRINT THE STATISTICS, FILL IN THE DETAILS BY PUTTING VARIBALE NAMES. DO NOT CHANGE THE FORMAT OF PRINTED OUTPUT */
  printf("\n\n===============STATISTICS======================= \n\n");
  printf("Number of original packets transmitted by A: %f \n", A_to_B);
  printf("Number of retransmissions by A: %f \n", A_retrans_B);
  printf("Number of data packets delivered to layer 5 at B: %f \n", B_to_layer5);
  printf("Number of ACK packets sent by B: %f \n", B_acks);
  printf("Number of corrupted packets: %f \n", corrupt);
  printf("Ratio of lost packets: %f \n", (A_retrans_B - corrupt) / ((A_to_B + A_retrans_B) + B_acks));
  printf("Ratio of corrupted packets: %f \n", (corrupt) / ((A_to_B + A_retrans_B) + B_acks - (A_retrans_B - corrupt)));
  printf("Average RTT: <YourVariableHere> \n");
  printf("Average communication time: <YourVariableHere> \n");
  printf("==================================================");

  /* PRINT YOUR OWN STATISTIC HERE TO CHECK THE CORRECTNESS OF YOUR PROGRAM */
  printf("\nEXTRA: \n");
  /* EXAMPLE GIVEN BELOW */
  /* printf("Example statistic you want to check e.g. number of ACK packets received by A : <YourVariableHere>"); */
}

/*****************************************************************
***************** NETWORK EMULATION CODE STARTS BELOW ***********
The code below emulates the layer 3 and below network environment:
  - emulates the tranmission and delivery (possibly with bit-level corruption
    and packet loss) of packets across the layer 3/4 interface
  - handles the starting/stopping of a timer, and generates timer
    interrupts (resulting in calling students timer handler).
  - generates message to be sent (passed from later 5 to 4)

THERE IS NOT REASON THAT ANY STUDENT SHOULD HAVE TO READ OR UNDERSTAND
THE CODE BELOW.  YOU SHOLD NOT TOUCH, OR REFERENCE (in your code) ANY
OF THE DATA STRUCTURES BELOW.  If you're interested in how I designed
the emulator, you're welcome to look at the code - but again, you should have
to, and you defeinitely should not have to modify
******************************************************************/

struct event
{
  double evtime;      /* event time */
  int evtype;         /* event type code */
  int eventity;       /* entity where event occurs */
  struct pkt *pktptr; /* ptr to packet (if any) assoc w/ this event */
  struct event *prev;
  struct event *next;
};
struct event *evlist = NULL; /* the event list */

/* Advance declarations. */
void init(void);
void generate_next_arrival(void);
void insertevent(struct event *p);

/* possible events: */
#define TIMER_INTERRUPT 0
#define FROM_LAYER5 1
#define FROM_LAYER3 2

#define OFF 0
#define ON 1

int TRACE = 1; /* for debugging purpose*/
FILE *fileoutput;
double time_now = 0.000;
int WINDOW_SIZE;
int LIMIT_SEQNO;
double RXMT_TIMEOUT;
double lossprob;    /* probability that a packet is dropped  */
double corruptprob; /* probability that one bit is packet is flipped */
double lambda;      /* arrival rate of messages from layer 5 */
int ntolayer3;      /* number sent into layer 3 */
int nlost;          /* number lost in media */
int ncorrupt;       /* number corrupted by media*/
int nsim = 0;
int nsimmax = 0;

unsigned int seed[5]; /* seed used in the pseudo-random generator */

int main(int argc, char **argv)
{
  struct event *eventptr;
  struct msg msg2give;
  struct pkt pkt2give;

  int i, j;

  init();
  A_init();
  B_init();

  while (1)
  {
    eventptr = evlist; /* get next event to simulate */
    if (eventptr == NULL)
      goto terminate;
    evlist = evlist->next; /* remove this event from event list */
    if (evlist != NULL)
      evlist->prev = NULL;
    if (TRACE >= 2)
    {
      printf("\nEVENT time: %f,", eventptr->evtime);
      printf("  type: %d", eventptr->evtype);
      if (eventptr->evtype == 0)
        printf(", timerinterrupt  ");
      else if (eventptr->evtype == 1)
        printf(", fromlayer5 ");
      else
        printf(", fromlayer3 ");
      printf(" entity: %d\n", eventptr->eventity);
    }
    time_now = eventptr->evtime; /* update time to next event time */
    if (eventptr->evtype == FROM_LAYER5)
    {
      generate_next_arrival(); /* set up future arrival */
                               /* fill in msg to give with string of same letter */
      j = nsim % 26;
      for (i = 0; i < 20; i++)
        msg2give.data[i] = 97 + j;
      msg2give.data[19] = '\n';
      nsim++;
      if (nsim == nsimmax + 1)
        break;
      A_output(msg2give);
    }
    else if (eventptr->evtype == FROM_LAYER3)
    {
      pkt2give.seqnum = eventptr->pktptr->seqnum;
      pkt2give.acknum = eventptr->pktptr->acknum;
      pkt2give.checksum = eventptr->pktptr->checksum;
      for (i = 0; i < 5; i++)
      {
        pkt2give.sack[i] = eventptr->pktptr->sack[i];
      }
      for (i = 0; i < 20; i++)
        pkt2give.payload[i] = eventptr->pktptr->payload[i];
      if (eventptr->eventity == A) /* deliver packet by calling */
        A_input(pkt2give);         /* appropriate entity */
      else
        B_input(pkt2give);
      free(eventptr->pktptr); /* free the memory for packet */
    }
    else if (eventptr->evtype == TIMER_INTERRUPT)
    {
      A_timerinterrupt();
    }
    else
    {
      printf("INTERNAL PANIC: unknown event type \n");
    }
    free(eventptr);
  }
terminate:
  Simulation_done(); /* allow students to output statistics */
  printf("Simulator terminated at time %.12f\n", time_now);
  return (0);
}

void init(void) /* initialize the simulator */
{
  int i = 0;
  printf("---- * Network Simulator Version 1.1 * ------ \n\n");
  printf("Enter number of messages to simulate: ");
  scanf("%d", &nsimmax);
  printf("Enter packet loss probability [enter 0.0 for no loss]:");
  scanf("%lf", &lossprob);
  printf("Enter packet corruption probability [0.0 for no corruption]:");
  scanf("%lf", &corruptprob);
  printf("Enter average time between messages from sender's layer5 [ > 0.0]:");
  scanf("%lf", &lambda);
  printf("Enter window size [>0]:");
  scanf("%d", &WINDOW_SIZE);
  LIMIT_SEQNO = WINDOW_SIZE * 2; // set appropriately; assumes SR here!
  printf("Enter retransmission timeout [> 0.0]:");
  scanf("%lf", &RXMT_TIMEOUT);
  printf("Enter trace level:");
  scanf("%d", &TRACE);
  printf("Enter random seed: [>0]:");
  scanf("%d", &seed[0]);
  for (i = 1; i < 5; i++)
    seed[i] = seed[0] + i;
  fileoutput = fopen("OutputFile", "w");
  if (fileoutput < (void *)0)
    exit(1);
  ntolayer3 = 0;
  nlost = 0;
  ncorrupt = 0;
  time_now = 0.0;          /* initialize time to 0.0 */
  generate_next_arrival(); /* initialize event list */
}

/****************************************************************************/
/* mrand(): return a double in range [0,1].  The routine below is used to */
/* isolate all random number generation in one location.  We assume that the*/
/* system-supplied rand() function return an int in therange [0,mmm]        */
/*      modified by Chong Wang on Oct.21, 2005                              */
/****************************************************************************/
int nextrand(int i)
{
  seed[i] = seed[i] * 1103515245 + 12345;
  return (unsigned int)(seed[i] / 65536) % 32768;
}

double mrand(int i)
{
  double mmm = 32767;    /* largest int  - MACHINE DEPENDENT!!!!!!!!   */
  double x;              /* individual students may need to change mmm */
  x = nextrand(i) / mmm; /* x should be uniform in [0,1] */
  if (TRACE == 0)
    printf("%.16f\n", x);
  return (x);
}

/********************* EVENT HANDLINE ROUTINES *******/
/*  The next set of routines handle the event list   */
/*****************************************************/
void generate_next_arrival(void)
{
  double x, log(), ceil();
  struct event *evptr;

  if (TRACE > 2)
    printf("          GENERATE NEXT ARRIVAL: creating new arrival\n");

  x = lambda * mrand(0) * 2; /* x is uniform on [0,2*lambda] */
                             /* having mean of lambda        */
  evptr = (struct event *)malloc(sizeof(struct event));
  evptr->evtime = time_now + x;
  evptr->evtype = FROM_LAYER5;
  evptr->eventity = A;
  insertevent(evptr);
}

void insertevent(event *p)
{
  struct event *q, *qold;

  if (TRACE > 2)
  {
    printf("            INSERTEVENT: time is %f\n", time_now);
    printf("            INSERTEVENT: future time will be %f\n", p->evtime);
  }
  q = evlist; /* q points to header of list in which p struct inserted */
  if (q == NULL)
  { /* list is empty */
    evlist = p;
    p->next = NULL;
    p->prev = NULL;
  }
  else
  {
    for (qold = q; q != NULL && p->evtime > q->evtime; q = q->next)
      qold = q;
    if (q == NULL)
    { /* end of list */
      qold->next = p;
      p->prev = qold;
      p->next = NULL;
    }
    else if (q == evlist)
    { /* front of list */
      p->next = evlist;
      p->prev = NULL;
      p->next->prev = p;
      evlist = p;
    }
    else
    { /* middle of list */
      p->next = q;
      p->prev = q->prev;
      q->prev->next = p;
      q->prev = p;
    }
  }
}

void printevlist(void)
{
  struct event *q;
  printf("--------------\nEvent List Follows:\n");
  for (q = evlist; q != NULL; q = q->next)
  {
    printf("Event time: %f, type: %d entity: %d\n", q->evtime, q->evtype, q->eventity);
  }
  printf("--------------\n");
}

/********************** Student-callable ROUTINES ***********************/

/* called by students routine to cancel a previously-started timer */
void stoptimer(int AorB)
{
  struct event *q /* ,*qold */;
  if (TRACE > 2)
    printf("          STOP TIMER: stopping timer at %f\n", time_now);
  /* for (q=evlist; q!=NULL && q->next!=NULL; q = q->next)  */
  for (q = evlist; q != NULL; q = q->next)
    if ((q->evtype == TIMER_INTERRUPT && q->eventity == AorB))
    {
      /* remove this event */
      if (q->next == NULL && q->prev == NULL)
        evlist = NULL;          /* remove first and only event on list */
      else if (q->next == NULL) /* end of list - there is one in front */
        q->prev->next = NULL;
      else if (q == evlist)
      { /* front of list - there must be event after */
        q->next->prev = NULL;
        evlist = q->next;
      }
      else
      { /* middle of list */
        q->next->prev = q->prev;
        q->prev->next = q->next;
      }
      free(q);
      return;
    }
  printf("Warning: unable to cancel your timer. It wasn't running.\n");
}

void starttimer(int AorB, double increment)
{

  struct event *q;
  struct event *evptr;

  if (TRACE > 2)
    printf("          START TIMER: starting timer at %f\n", time_now);
  /* be nice: check to see if timer is already started, if so, then  warn */
  /* for (q=evlist; q!=NULL && q->next!=NULL; q = q->next)  */
  for (q = evlist; q != NULL; q = q->next)
    if ((q->evtype == TIMER_INTERRUPT && q->eventity == AorB))
    {
      printf("Warning: attempt to start a timer that is already started\n");
      return;
    }

  /* create future event for when timer goes off */
  evptr = (struct event *)malloc(sizeof(struct event));
  evptr->evtime = time_now + increment;
  evptr->evtype = TIMER_INTERRUPT;
  evptr->eventity = AorB;
  insertevent(evptr);
}

/************************** TOLAYER3 ***************/
void tolayer3(int AorB, pkt packet)
{
  struct pkt *mypktptr;
  struct event *evptr, *q;
  double lastime, x;
  int i;

  ntolayer3++;

  /* simulate losses: */
  if (mrand(1) < lossprob)
  {
    nlost++;
    if (TRACE > 0)
      printf("          TOLAYER3: packet being lost\n");
    return;
  }

  /* make a copy of the packet student just gave me since he/she may decide */
  /* to do something with the packet after we return back to him/her */
  mypktptr = (struct pkt *)malloc(sizeof(struct pkt));
  mypktptr->seqnum = packet.seqnum;
  mypktptr->acknum = packet.acknum;
  mypktptr->checksum = packet.checksum;
  for (i = 0; i < 5; i++)
  {
    mypktptr->sack[i] = packet.sack[i];
  }
  for (i = 0; i < 20; i++)
    mypktptr->payload[i] = packet.payload[i];
  if (TRACE > 2)
  {
    printf("          TOLAYER3: seq: %d, ack %d, check: %d ", mypktptr->seqnum,
           mypktptr->acknum, mypktptr->checksum);
  }

  /* create future event for arrival of packet at the other side */
  evptr = (struct event *)malloc(sizeof(struct event));
  evptr->evtype = FROM_LAYER3;      /* packet will pop out from layer3 */
  evptr->eventity = (AorB + 1) % 2; /* event occurs at other entity */
  evptr->pktptr = mypktptr;         /* save ptr to my copy of packet */
                                    /* finally, compute the arrival time of packet at the other end.
                                       medium can not reorder, so make sure packet arrives between 1 and 10
                                       time units after the latest arrival time of packets
                                       currently in the medium on their way to the destination */
  lastime = time_now;
  /* for (q=evlist; q!=NULL && q->next!=NULL; q = q->next) */
  for (q = evlist; q != NULL; q = q->next)
    if ((q->evtype == FROM_LAYER3 && q->eventity == evptr->eventity))
      lastime = q->evtime;
  evptr->evtime = lastime + 1 + 9 * mrand(2);

  /* simulate corruption: */
  if (mrand(3) < corruptprob)
  {
    ncorrupt++;
    if ((x = mrand(4)) < 0.75)
      mypktptr->payload[0] = '?'; /* corrupt payload */
    else if (x < 0.875)
      mypktptr->seqnum = 999999;
    else
      mypktptr->acknum = 999999;
    if (TRACE > 0)
      printf("          TOLAYER3: packet being corrupted\n");
  }

  if (TRACE > 2)
    printf("          TOLAYER3: scheduling arrival on other side\n");
  insertevent(evptr);
}

void tolayer5(char datasent[20])
{
  fwrite(datasent, 1, 20, fileoutput);
}