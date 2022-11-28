.text
.globl main            		# label "main" must be global

main:

label:  add $9, $8, $4  	#$9 = 12
sw $t1, 5($t1)  
lw $s2, 5($t1)
        sw $9, 5($11)     	# data[16] = 12
        lw $18, 5($11),   	 #$18 = 12
        slt $9, $8, $19  	#$9 = 1
        addi $16, $16, 1 	#$16 = 17
        slt $17, $5, $10 	#$17 = 1
        bne $16, $1, label2	# pame label 2 tin proti fora
label3: beq $9, $1, label	#pame label1
label2: addi $16,$16,-1		#$16 = 16
	lw $15, 5($11)		#$15 = 12
	and $24, $15, $7	#$24 = 4
	or $19, $5, $6		#$19 = 7
	slt $16,$2,$24		#$16 = 1
	beq $16, $1, label3	# pame label 3