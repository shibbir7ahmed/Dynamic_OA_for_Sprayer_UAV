%% Find Total Path Length
PathLength = 0;

Speed = 0*out.time;
for i = 1:length(Speed)
    Speed(i) = sqrt((out.VXYZ(i,1))^2 + (out.VXYZ(i,2))^2 + (out.VXYZ(i,3))^2 );
    PathLength = PathLength + Speed(i)*0.1;
end

%% Plot XYZ vs Time
figure;
subplot(3,1,1);
plot(out.time,out.XYZ(:,1));
xlabel('time');
ylabel('X Pos');

subplot(3,1,2);
plot(out.time,out.XYZ(:,2));
xlabel('time');
ylabel('Y Pos');

subplot(3,1,3);
plot(out.time,out.XYZ(:,3));
xlabel('time');
ylabel('Z Pos');

%% Plot Trajectory % Obstacle
figure;
% Drone Trajectory
h1 = scatter(out.XYZ(:,1),out.XYZ(:,2),10); hold on;
% Obstacle
DrawRecObs(Obs); hold on;

SprayLength = 3;

pgon = polyshape([0,0,0.1,0.1],[0,0.1,0.1,0]);
area_resolution = 2;
for i = 51:area_resolution:length(out.time)
    Px = out.XYZ(i,1);
    Py = out.XYZ(i,2);
    p1 = [Px+1.6,Py-1.6];
    p2 = [Px-1.6,Py-1.6];
    p3 = [Px-1.6,Py+1.6];
    p4 = [Px+1.6,Py+1.6];
    
    pgon1 = polyshape([p1(1), p2(1), p3(1), p4(1)],[p1(2), p2(2), p3(2), p4(2)]);
    pgon = union(pgon,pgon1);

    %patch([p1(1), p2(1),p3(1),p4(1)],[p1(2), p2(2),p3(2),p4(2)],'r','EdgeColor','none'); hold on
end
hold on;
h2 = plot(pgon,'FaceColor','red','FaceAlpha',0.5);
hold on;
h3 = plot([-20-1.6 20+1.6],[0+1.6 0+1.6],'b','Linewidth',3); hold on;
plot([20+1.6 20+1.6],[0+1.6 -12-1.6],'b','Linewidth',3); hold on;
plot([20+1.6 -20-1.6],[-12-1.6 -12-1.6],'b','Linewidth',3); hold on;
plot([-20-1.6 -20-1.6],[0+1.6 -12-1.6],'b','Linewidth',3);

daspect([1 1 1])
xlim([-25, 25])
ylim([-17 5])
%%camroll(-270)
grid on;
grid minor;
xlabel('X')
ylabel('Y')
title('Area Coverage (3 meter gap mission, '+string(num2str(out.LiquidLevel*100))+' 10% Liquid Level');

legend([h1 h2 h3],'Drone Trajectory','Actual Area Coverage','Ideal Area Coverage');

CoverArea = area(pgon);
TotalArea = (40 + 2*1.6)*(12 + 2*1.6);
CoverAreaPercentage = 100*CoverArea/TotalArea;

%% Plot Sensor Distance
figure
subplot(3,2,1);
plot(out.time,out.d0X);
ylabel('dc (m)');
xlabel('time (sec)')
grid on;

subplot(3,2,2);
plot(out.time,out.d0Y);
ylabel('db (m)');
grid on;

subplot(3,2,3);
plot(out.time,out.d1);
ylabel('d1 (m)');
grid on;

subplot(3,2,4);
plot(out.time,out.d3);
ylabel('d3 (m)');
grid on;

subplot(3,2,5);
plot(out.time,out.d2);
ylabel('d2 (m)');
grid on;

subplot(3,2,6);
plot(out.time,out.d4);
ylabel('d4 (m)');
grid on;


%% Command Plot
figure
subplot(2,1,1)
plot(out.time,out.ForwardVelCmd);
ylabel('Xb Velocity Cmd (m/s)')
xlabel('time (sec)')
grid on;

subplot(2,1,2)
plot(out.time,out.SideVelCmd);
ylabel('Yb Velocity Cmd (m/s)');
xlabel('time (sec)')
grid on

%% 3D Animation

figure;
ViewSize = 5;
xlimit = [-21 21];
ylimit = [-21 21];
zlimit = [0 6];
width = 800;
height = 750;
NewFigure(xlimit,ylimit,zlimit,-90,90,width,height);
%%camroll(270)
pause(1)
AnimEulerTar(ViewSize,out.time,out.XYZ,out.EulerAngles,out.VXYZ,Obs,...
             out.Sensor0Vect,...
             out.SensorX1Pos,out.SensorX1Vect,out.SensorX2Pos,out.SensorX2Vect,...
             out.SensorY1Pos,out.SensorY1Vect,out.SensorY2Pos,out.SensorY2Vect,out.LiquidLevel,...
             out.mode,out.WayPts,...
             out.Sensor0RVect,out.Sensor0LVect);
                          

clear xlimit;
clear ylimit;
clear zlimit;
clear width;
clear height;

%% Local Functions

function NewFigure(xlim,ylim,zlim,viewx,viewy,w,h)
    %set(gca, 'XLim', xlim,'YLim',ylim,'ZLim',zlim);
    view(viewx,viewy)
    x0=10;
    y0=10;
    set(gcf,'position',[x0,y0,w,h])
    hold on;
    grid on;
    grid minor;
end

function AnimEulerTar(ViewSize,t_plot,XYZs,EulerAngles,VXYZs,Obs,...
                      Sensor0Vect,...
                      SensorX1Pos,SensorX1Vect,SensorX2Pos,SensorX2Vect,...
                      SensorY1Pos,SensorY1Vect,SensorY2Pos,SensorY2Vect,...
                      WaterLevel,mode,WayPts,...
                      Sensor0RVect,Sensor0LVect)
                  
    t_section = 0;
    curve = animatedline('LineWidth',1.5);
    
    for i = 1:size(WayPts,1)
        Pts = WayPts(i,:);
        scatter3(Pts(1),Pts(2),Pts(3),'filled','MarkerFaceColor','b','MarkerEdgeColor','b');
        hold on;
    end
    
    % Draw Rectangle Obstacle Boundary
    DrawRecObs(Obs)

    % Draw Initial Position
    scatter3(XYZs(1,1),XYZs(1,2),XYZs(1,3),'filled','MarkerFaceColor','black','MarkerEdgeColor','black')
    xlabel('X')
    ylabel('Y')
    
    for i = 1:length(t_plot)
        if abs( t_plot(i) - t_section) < 0.0001
            % Do Animation
            Euler = EulerAngles(i,:)
            XYZ = XYZs(i,:)
            VXYZ = VXYZs(i,:)
            md = mode(i);
            
            S0V = Sensor0Vect(i,:);
            
            S0RV = Sensor0RVect(i,:);
            S0LV = Sensor0LVect(i,:);
            
            SX1P = SensorX1Pos(i,:);
            SX1V = SensorX1Vect(i,:);
            SX2P = SensorX2Pos(i,:);
            SX2V = SensorX2Vect(i,:);
            SY1P = SensorY1Pos(i,:);
            SY1V = SensorY1Vect(i,:);
            SY2P = SensorY2Pos(i,:);
            SY2V = SensorY2Vect(i,:);
            
            O = eye(3);
            T_BtoI = matrixB2I(Euler(1),Euler(2),Euler(3));
            O_I = T_BtoI*O;
            
            addpoints(curve, XYZ(1), XYZ(2),XYZ(3));

                 
            %line1 = drawline(XYZ,O_I(:,1),'b--',1.5)
            %line2 = drawline(XYZ,O_I(:,2),'g--',1.5)
            %line3 = drawline(XYZ,O_I(:,3),'r--',1.5)
            %line5 = extendline(XYZ,O_I(:,1),'b:')
            
            % Sensor Line
            line_S0 = drawline(XYZ,transpose(S0V),'r-.',2);
            line_S0R = drawline(XYZ,transpose(S0RV),'r--',1);
            line_S0L = drawline(XYZ,transpose(S0LV),'r--',1);
            line_SX1 = drawline(SX1P,transpose(SX1V),'b--',2);
            line_SX2 = drawline(SX2P,transpose(SX2V),'b--',2);
            line_SY1 = drawline(SY1P,transpose(SY1V),'g--',2);
            line_SY2 = drawline(SY2P,transpose(SY2V),'g--',2);
  
            
            frame1 = drawline(XYZ,0.5/sqrt(2)*O_I(:,1)+0.5/sqrt(2)*O_I(:,2),'black',2.5);
            frame2 = drawline(XYZ,0.5/sqrt(2)*O_I(:,1)-0.5/sqrt(2)*O_I(:,2),'black',2.5);
            frame3 = drawline(XYZ,-0.5/sqrt(2)*O_I(:,1)+0.5/sqrt(2)*O_I(:,2),'black',2.5);
            frame4 = drawline(XYZ,-0.5/sqrt(2)*O_I(:,1)-0.5/sqrt(2)*O_I(:,2),'black',2.5);
            
            velline = drawline(XYZ,transpose(VXYZ),'c',2);
            
            axis equal;
            xlim([-ViewSize ViewSize]+XYZ(1));
            ylim([-ViewSize ViewSize]+XYZ(2));
            %axis equal;
            
            
            
            
            % logs     
            title('Tank Percent: '+ string(num2str(WaterLevel*100)) + ' %, Time: '...
                  + string( num2str(t_plot(i),'%.1f') )+' sec, Mode: ' + string(num2str(md)) );
            logtxt0 = 'Tank Percent: '+ string(num2str(WaterLevel*100)) + ' %, Time: '...
                  + string( num2str(t_plot(i),'%.1f') )+' sec, Mode: ' + string(num2str(md));
            
            dispstr7 = string( num2str( VXYZ(1),'%.1f' ) );
            dispstr8 = string( num2str( VXYZ(2),'%.1f' ) );
            dispstr9 = string( num2str( VXYZ(3),'%.1f' ) );
            vstr = 'Velocity ['+dispstr7+ ' , '+dispstr8 + ' , ' + dispstr9 + ']';
            
            dispstr1 = string( num2str( rad2deg(Euler(1)),'%.1f' ) );
            dispstr2 = string( num2str( rad2deg(Euler(2)),'%.1f' ) );
            dispstr3 = string( num2str( rad2deg(Euler(3)),'%.1f' ) );
            dispstr4 = string( num2str( XYZ(1),'%.1f' ) );
            dispstr5 = string( num2str( XYZ(2),'%.1f' ) );
            dispstr6 = string( num2str( XYZ(3),'%.1f' ) );
            logtxt = 'EulerAngle ['+dispstr1+' , '+dispstr2+' , '+dispstr3+']  XYZ [' + dispstr4...
                + ' , ' +dispstr5+ ' , '+dispstr6 +']   ' + vstr;         
            TXT = text(XYZ(1)-5,XYZ(2)-7,logtxt0+newline+logtxt);
            
            
            drawnow
            pause(0.05)
            t_section = t_section + 0.2;            
            
            delete(line_S0)           
            delete(line_S0R)
            delete(line_S0L)
            
            delete(line_SX1)
            delete(line_SX2)
            delete(line_SY1)
            delete(line_SY2)
                
            delete(frame1)
            delete(frame2)
            delete(frame3)
            delete(frame4)
            
            delete(velline)
            delete(TXT)
                
        end
   
    end

end

function SubAnimEuler(t_plot,XYZs,EulerAngles,TR)
    figure;
    t_section = 0  
    for i = 1:length(t_plot)
        if abs( t_plot(i) - t_section) < 0.0001
            % Do Animation    
            subplot(1,2,1)
            NewFigure([-3 3],[-1 5],[0 6],0,0,1200,600);
            scatter3(TR(1),TR(2),TR(3),'filled','MarkerFaceColor','black','MarkerEdgeColor','black')       
            Euler = EulerAngles(i,:)
            XYZ = XYZs(i,:)
            O = eye(3);
            T_BtoI = matrixB2I(Euler(1),Euler(2),Euler(3));
            O_I = T_BtoI*O
            line1 = drawline(XYZ,O_I(:,1),'b')
            line2 = drawline(XYZ,O_I(:,2),'g')
            line3 = drawline(XYZ,O_I(:,3),'r')
            line4 = extendline(XYZ,O_I(:,3),'r--')
            line5 = extendline(XYZ,O_I(:,1),'b:')
            
            % labels
            title('XZ plane (side view)')
            dispstr1 = string( num2str( rad2deg(Euler(1)),'%.1f' ) );
            dispstr2 = string( num2str( rad2deg(Euler(2)),'%.1f' ) );
            dispstr3 = string( num2str( rad2deg(Euler(3)),'%.1f' ) );
            dispstr4 = string( num2str( XYZ(1),'%.1f' ) );
            dispstr5 = string( num2str( XYZ(2),'%.1f' ) );
            dispstr6 = string( num2str( XYZ(3),'%.1f' ) );            
            mystr = string( num2str(t_plot(i),'%.1f'))+' sec 3  EulerAngle ['+dispstr1+' , '+dispstr2+' , '+dispstr3+']  XYZ [' + dispstr4...
            + ' , ' +dispstr5+ ' , '+dispstr6 +']';
            txt1 = annotation('textbox', [0.35, 0.9, 0.1, 0.1], 'string', mystr)

            
            subplot(1,2,2)
            NewFigure([-3 3],[-1 5],[0 6],0,90,1200,600);
            scatter3(TR(1),TR(2),TR(3),'filled','MarkerFaceColor','black','MarkerEdgeColor','black')       
            line6 = drawline(XYZ,O_I(:,1),'b')
            line7 = drawline(XYZ,O_I(:,2),'g')
            line8 = drawline(XYZ,O_I(:,3),'r')
            line9 = extendline(XYZ,O_I(:,3),'r--')
            line10 = extendline(XYZ,O_I(:,1),'b:')
            % labels
            title('XY plane (top view)')

            
            drawnow
            pause(0.01)        
            if i ~= length(EulerAngles)
                delete(line1)
                delete(line2)
                delete(line3)
                delete(line4)
                delete(line5)
                delete(line6)
                delete(line7)
                delete(line8)
                delete(line9)
                delete(line10)
                delete(txt1)
            end
                        
            t_section = t_section + 0.2
        end
    end    
end

function m = matrixB2I(phi,theta,psi)
    T_BtoV2 = [[1 0 0];[0 cos(-phi) sin(-phi)];[0 -sin(-phi) cos(-phi)]];
    T_V2toV1 = [[cos(-theta) 0 -sin(-theta)];[0 1 0];[sin(-theta) 0 cos(-theta)]];
    T_V1toI = [[cos(-psi) sin(-psi) 0];[-sin(-psi) cos(-psi) 0];[0 0 1]];
    m = T_V1toI*T_V2toV1*T_BtoV2;
end

function line = drawline(p1,p2,color,width)
% MYMEAN Local function that calculates mean of array.
    pt1 = p1
    pt2 = pt1 + transpose(p2);
    pts = [pt1;pt2];
    line = plot3(pts(:,1), pts(:,2), pts(:,3),color,'LineWidth',width);
end

function DrawRecObs(OBs)

    for j = 1:size(OBs,2)
        Ob = OBs{1,j};
        for k = 1:size(Ob,1)
            if k == size(Ob,1)
                Pos1 = Ob(k,:);
                Pos2 = Ob(1,:);
            else
                Pos1 = Ob(k,:);
                Pos2 = Ob(k+1,:);
            end
            p1 = [Pos1(1),Pos1(2),0];
            p4 = [Pos1(1),Pos1(2),6];
            p2 = [Pos2(1),Pos2(2),0];
            p3 = [Pos2(1),Pos2(2),6];
            patch([p1(1), p2(1),p3(1),p4(1)],[p1(2), p2(2),p3(2),p4(2)],[p1(3), p2(3),p3(3),p4(3)],'y');      
        end
    
        xlist = transpose(Ob(:,1));
        ylist = transpose(Ob(:,2));
        zlist = 0*xlist;
        patch(xlist,ylist,zlist,'y');
  
        alpha(0.3);
    end
end

function line = extendline(p1,p2,color)
% MYMEAN Local function that calculates mean of array.
    pt1 = p1
    pt2 = pt1 + 20*transpose(p2);
    pts = [pt1;pt2];
    line = plot3(pts(:,1), pts(:,2), pts(:,3),color,'LineWidth',1.5);
end

function VisAttitude(Euler,linsty)
    O = eye(3);
    T_BtoI = matrixB2I(Euler(1),Euler(2),Euler(3));
    O_I = T_BtoI*O
    for i = 1:length(O_I)
        drawline(O_I(:,i),linsty)
    end
    z = O_I(:,3)
    scatter3(z(1),z(2),z(3),'filled','MarkerFaceColor','b','MarkerEdgeColor','b')
end


