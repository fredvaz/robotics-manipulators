
%% Anima o square através de do desenho constante do mesmo!

function animateSquare(TA, TB, TC, TD, TE, TF)

    global delay squareA squareB squareC squareD squareE squareF

    hold off
    pause(delay)

    square1 = transPointsSquare(TA, squareA);
    drawSquare(square1, 'b');

    square2 = transPointsSquare(TB, squareB);
    drawSquare(square2, 'r');

    square3 = transPointsSquare(TC, squareC);
    drawSquare(square3, 'b');

    square4 = transPointsSquare(TD, squareD);
    drawSquare(square4, 'r');
    
    square5 = transPointsSquare(TE, squareE);
    drawSquare(square5, 'g');
    
    square6 = transPointsSquare(TF, squareF);
    drawSquare(square6, 'g');

end