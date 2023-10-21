%% Create two robots
baseTr_1 = transl(-0.4, 0, 0);
baseTr_2 = transl(0.4, 0, 0)*trotz(pi);

bot1 = DobotMagician(baseTr_1);
bot2 = DobotMagician(baseTr_2);

bots_ = {bot1, bot2};

%% Create interface instance
interface_ = MULTIBOTGUI(bots_);

%% Function testing
%jointTest_ = [0,1,1,1,1];
%interface_.moveRobot(jointTest_, 1)