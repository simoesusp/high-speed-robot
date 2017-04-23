#define MaxPayload 32// Maximum message size (32 bytes)

void append(char *, char *);
void copyString(char *, char *);
bool isSubstring(char *, char *);
bool isEqual(char *, char *);
int char2int(char*);
void int2char(char *o,int n);
int mypow(int , int );



void append(char *o, char *p)
{
	int i,j,k;
	for(i = 0; i < MaxPayload; i++)
		if(o[i] == '\0')
			break;

	for(j = 0; j < MaxPayload; j++)
		if(p[j] == '\0')
			break;

	if( i+j <= MaxPayload)
	{
		for(k = 0; k < j ;k++)
			o[i+k] = p[k];
		o[i+k] = '\0';
	}
}

void copyString(char *o, char *c)
{
    int i;
    for(i = 0; i < MaxPayload; i++)
    {
       c[i] = o[i];
        if(o[i] == '\0')
            break;
    }
}

bool isSubstring(char *a, char *b)
{
	int i;

	if((a[0] == '\0')||(b[0] == '\0'))
	{
		return 0;
	}	

	for(i = 0; i < MaxPayload; i++ )
	{
		if((a[i] == '\0')||(b[i] == '\0'))
				return 1;
		if(a[i] != b[i])
			return 0;
	}
	return 1;
}


bool isEqual(char *a, char *b)
{
	int i;

	if((a[0] == '\0')||(b[0] == '\0'))
	{
		return 0;
	}	

	for(i = 0; i < MaxPayload; i++ )
	{
		if((a[i] == '\0')||(b[i] == '\0'))
			if((a[i] == '\0')&&(b[i] == '\0'))
				return 1;
			else
			{
				return 0;
			}
		if(a[i] != b[i])
			return 0;
	}
	return 1;
}

int char2int(char *s)
{

    /*
       int r = 0;

       if(s[1] == '\0')
       {
       if(s[0] >= '0' && s[0] <= '9')
       {
       r = (s[0] - 48);
       return r;
       }
       return -1;
       }

       if(s[2] == '\0')
       {
       if(s[0] >= '0' && s[0] <= '9' && s[1] >= '0' && s[1] <= '9')
       {
       r = (s[1] - 48) + (s[0] - 48)*10;
       return r;
       }
       return -1;
       }

       if(s[3] == '\0')
       {
       if(s[0] >= '0' && s[0] <= '9' && s[1] >= '0' && s[1] <= '9' && s[2] >= '0' && s[2] <= '9')
       {
       r = (s[2] - 48) + (s[1] - 48)*10 + (s[0] - 48)*100 ;
       return r;
       }
       return -1;
       }

       return -2;
     */

    int i = 0,j, r = 0, aux;

    for(i = 0; i < MaxPayload; i++)
    {
        if( !((s[i] >= '0') && (s[i] <= '9')) ) // is it a number?
        {
            if(s[i] == '\0')
                break;
            else
                return -1;
        }
    }

    if(i == MaxPayload) // number too big for this application
        return -3;

    if(i == 0) // empty string
        return -2;

    for(j=0; j < i; j++)
    {
        aux = (int) s[j];
        aux -= 48;
        aux *= (int) mypow(10,i-1-j);
        r += aux;
    }
    return r;
}

void int2char(char *o,int n)
{
	int i = 1, aux = 10, j;
	if(n >= 0)
	{
		while(n/i > 9)
			i *=10;
		for(j = 0; i>0; j++)
		{
			aux = n/i;
			o[j] = aux+48;
			n -= aux*i;
			i /= 10;
		}
		o[j] = '\0';

	}
	else // negative values aren't useful for this application
		o[0] = '\0';
}

int mypow(int base, int exponent)
{
    int result = 1;
    for(int i = 0; i < exponent; i++)
        result = result*base;
    return result;
}

