        .area CSEG    (ABS,CODE)
; 	r7	is 	(flash address)/4 hi byte
; 	r6	is 	(flash address)/4 lo byte
DMA0CFGH       =       0x00d5
DMA0CFGL       =       0x00d4
DMA1CFGH       =       0x00d3
DMA1CFGL       =       0x00d2
DMAARM =       0x00d6
FCTL   =       0x6270
FADDRL =       0x6271
FADDRH =       0x6272
FWRITE = 2

	.org 0
inp:
	.db	0x62
	.db	0x60
	.db	buff>>8	; buff
	.db	buff
	.db	0x02	; 512 bytes
	.db	0x00
	.db	0x1f
	.db	0x11
outp:
	.db	buff>>8	; buff
	.db	buff
	.db	0x62	; 
	.db	0x73
	.db	0x02	; 512 bytes
	.db	0x00
	.db	0x12
	.db	0x42
;start:	mov	DMA0CFGH, #inp>>8		
;	mov	DMA0CFGL, #inp			
;	mov 	dptr, #inp+2			
;	clr	a				
;	movx	@dptr, a				
;	inc	dptr				
;	mov	a, #buff			
;	movx	@dptr, a				
;	inc	dptr				
;	mov	a, #512>>8			
;	movx	@dptr, a				
;	inc	dptr				
;	clr	a			
;	movx	@dptr, a				
;
;	mov	DMAARM, #1			
;
;	mov 	dptr, #outp			
;	clr	a				
;	movx	@dptr, a				
;	inc	dptr				
;	mov	a, #buff			
;	movx	@dptr, a				
;	inc	dptr				
;	inc	dptr				
;	inc	dptr				
;	mov	a, #512>>8			
;	movx	@dptr, a				
;	inc	dptr				
;	clr	a
;	movx	@dptr, a				

;        mov     DMA1CFGH, #outp>>8		
;        mov     DMA1CFGL, #outp			
;	mov	dptr, #FADDRL			
;	mov	a, r6	; low address		
;	movx	@dptr, a			
;	inc	dptr				
;	mov	a, r7	; hi address		
;	movx	@dptr, a			
;
;	mov	dptr, #FCTL			
;
;l1:	mov	a, DMAARM	;  wait for in to complete
;	jb	a.0, l1		
;
;	mov	DMAARM, #2
;	mov	a, #FWRITE
;	movx	@dptr, a
;l2:	mov	a, DMAARM	; wait for out to complete
;	jb	a.1, l2
;l3:	movx	a, @dptr	; wait for flash to complete
;	jb	a.7, l3
;
;l4:	.db	0xA5    ; invalid?
;        sjmp l4
buff:	.db	0
