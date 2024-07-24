read X
read Y
read Z

if [ $X -eq $Y ] && [ $X -eq $Z ]
then
    echo "EQUILATERAL"
elif (( X == Y && X != Z )) || (( X == Z && X != Y ))
then    
    echo "ISOSCELES"
else
    echo "SCALENE"
fi
