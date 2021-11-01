#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/file.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>

/* ******************************************************************
   ARQ NETWORK EMULATOR: VERSION 1.1  J.F.Kurose
   MODIFIED by Chong Wang on Oct.21,2005 for csa2,csa3 environments

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
  char payload[20];

};

/*- Your Definitions
  ---------------------------------------------------------------------------*/

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

/* WINDOW_SIZE, RXMT_TIMEOUT and TRACE are inputs to the program;
   Please set an appropriate value for LIMIT_SEQNO.
   You have to use these variables in your
   routines --------------------------------------------------------------*/

extern int WINDOW_SIZE;     // size of the window
extern int LIMIT_SEQNO;     // when sequence number reaches this value,                                     // it wraps around
extern double RXMT_TIMEOUT; // retransmission timeout
extern int TRACE;           // trace level, for your debug purpose
extern double time_now;     // simulation time, for your debug purpose

/********* YOU MAY ADD SOME ROUTINES HERE ********/

struct window_frame {
  struct pkt packet;
  int timeout;
  int sent; 
  int acked;
} Frame;

struct window_frame send_window[1000];
struct window_frame rcv_window[1000];
struct pkt buffer[1000];
int buffer_i = 0;

enum Sender_State
{
  WAIT_ABOVE,
  WAIT_ACK

};

// enum Receiver_State{

// };

struct Sender
{
  int id;
  int seq;
  int send_base;
  enum Sender_State state;

} Sender_A;

struct Receiver
{
  int id;
  int seq;
  int rcv_base;

} Receiver_B;

/* compute the checksum by summing up all fields in the given packet */
int compute_checksum(struct pkt *packet)
{

  int checksum = 0;
  checksum += packet->seqnum;
  checksum += packet->acknum;

  for (int i = 0; packet->payload[i] != '\0'; ++i)
  {
    checksum += packet->payload[i];
  }
  return checksum;
}

void send_ack(int ack)
{

  struct pkt packet;
  packet.acknum = ack;
  packet.checksum = compute_checksum(&packet);
  tolayer3(B, packet);
}

int check_send_window()
{

  return Sender_A.seq >= Sender_A.send_base + WINDOW_SIZE;
}

void buffer_msg(struct msg message){



}

void deliver_packets(){




}



/********* STUDENTS WRITE THE NEXT SEVEN ROUTINES *********/

/* called from layer 5, passed the data to be sent to other side */
void
    A_output(message) struct msg message;
{

  struct pkt packet;
  packet.seqnum = Sender_A.seq;
  memmove(packet.payload, message.data, 20);
  packet.checksum = compute_checksum(&packet);

  if(!check_send_window()){
    buffer_msg(packet);
    return;
  }

  if (Sender_A.state == WAIT_ACK)
  {
    printf("Waiting for ACK ...");
  }

  packets_window[Sender_A.seq].packet = packet;
  packets_window[Sender_A.seq].sent = 1;
  packets_window[Sender_A.seq].acked = 0;
  // packets_window[Sender_A.seq].timeout
  Sender_A.state = WAIT_ACK;
  tolayer3(A, packet);
  ++Sender_A.seq;
  starttimer(A, RXMT_TIMEOUT);
}

/* called from layer 3, when a packet arrives for layer 4 */
void
    A_input(packet) struct pkt packet;
{
  if(packet.checksum != compute_checksum(&packet)){
    printf("Packet corrupted!");
    return;
  }


  packets_window[packet.seqnum].acked = 1;
  
  
}

/* called when A's timer goes off */
void A_timerinterrupt(void)
{
}

/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  Sender_A.id = A;
  Sender_A.seq = FIRST_SEQNO;
  Sender_A.state = WAIT_ABOVE;
  Sender_A.send_base = FIRST_SEQNO;
}

/* called from layer 3, when a packet arrives for layer 4 at B*/
void
    B_input(packet) struct pkt packet;
{
  if (packet.checksum != compute_checksum(&packet))
  {
    printf("corrupted packet!");
    send_ack(0);
    return;
  }

  send_ack(1);


  tolayer5(packet.payload);
}

/* the following rouytine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  Receiver_B.id = B;
  Receiver_B.seq = FIRST_SEQNO;
}

/* called at end of simulation to print final statistics */
void Simulation_done()
{
  /* TO PRINT THE STATISTICS, FILL IN THE DETAILS BY PUTTING VARIBALE NAMES. DO NOT CHANGE THE FORMAT OF PRINTED OUTPUT */
  printf("\n\n===============STATISTICS======================= \n\n");
  printf("Number of original packets transmitted by A: <YourVariableHere> \n");
  printf("Number of retransmissions by A: <YourVariableHere> \n");
  printf("Number of data packets delivered to layer 5 at B: <YourVariableHere> \n");
  printf("Number of ACK packets sent by B: <YourVariableHere> \n");
  printf("Number of corrupted packets: <YourVariableHere> \n");
  printf("Ratio of lost packets: <YourVariableHere> \n");
  printf("Ratio of corrupted packets: <YourVariableHere> \n");
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

int TRACE = 0; /* for debugging purpose */
int fileoutput;
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
  printf("----- * Network Simulator Version 1.1 * ------ \n\n");
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
  LIMIT_SEQNO = WINDOW_SIZE * 2; // set appropriately; here assumes SR
  printf("Enter retransmission timeout [> 0.0]:");
  scanf("%lf", &RXMT_TIMEOUT);
  printf("Enter trace level:");
  scanf("%d", &TRACE);
  printf("Enter random seed: [>0]:");
  scanf("%d", &seed[0]);
  for (i = 1; i < 5; i++)
    seed[i] = seed[0] + i;
  fileoutput = open("OutputFile", O_CREAT | O_WRONLY | O_TRUNC, 0644);
  if (fileoutput < 0)
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
/*     modified by Chong Wang on Oct.21,2005                                */
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
  //   char *malloc(); commented out by matta 10/17/2013

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

void
    insertevent(p) struct event *p;
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
void
    stoptimer(AorB) int AorB; /* A or B is trying to stop timer */
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

void
    starttimer(AorB, increment) int AorB; /* A or B is trying to stop timer */
double increment;
{

  struct event *q;
  struct event *evptr;
  // char *malloc(); commented out by matta 10/17/2013

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
void
    tolayer3(AorB, packet) int AorB; /* A or B is trying to stop timer */
struct pkt packet;
{
  struct pkt *mypktptr;
  struct event *evptr, *q;
  // char *malloc(); commented out by matta 10/17/2013
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
  /* modified by Chong Wang on Oct.21, 2005  */
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

void
    tolayer5(datasent) char datasent[20];
{
  write(fileoutput, datasent, 20);
}