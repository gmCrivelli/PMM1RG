# Author: Gustavo de Mello Crivelli
# Special thanks to Fabio Usberti, without whose unfailing support this would not have been possible
# PFG IC UNICAMP, 2019

import random
import math
import os
import networkx as nx
import matplotlib.pyplot as plt
import uuid
import json
import sys
import time

HOLE_TAG = 0
OBSTACLE_TAG = 1
ROBOT_TAG = 2

    # Algorithm:
    # Find path from robot R to goal T
    # Find Hf -> holes in front of the robot
    # Find Hb -> holes behind the robot
    # Find Hm -> Minimum holes needed to reach the goal T

    # Is problem possible with just Hf?
    # NO -> Assume we manage to use Hf + Hb holes. Is that enough?
    #   NO -> Then it's impossible.
    #   YES-> Let's check it out.
    #       A.1) Find the furthest reachable branch vertex Wb between the robot R and the goal T.
    #       A.2) Check If |Hf| >= distance(v(R),Wb) + 1:
    #           A.2.1) YES: We have guaranteed access to |Hf + Hb| holes by clever movement of the robot, which is enough to solve the problem.
    #               A.2.1.1) Each sidestep vertex Ws neighboring Wb has a value |Hw|, the number of holes in that branch (behind Ws). Pick Ws at random.
    #               A.2.1.2) By moving to Ws, we will have access to Hf + Hb - Hw holes (Also, we may have a new Hm' number of minimum holes, but ignore that for now). Check if Hf + Hb - Hw >= Hm.
    #               A.2.1.2.1) YES: Clear the path from R to Ws, then move R to Ws, and solve the new problem using (B.1).
    #               A.2.1.2.2) NO: Then K = Hm - Hf - Hb + Hw > 0 , and we need to move that many holes into the branch of Ws.
    #               A.2.1.2.2.1) Starting from the robot, pick the K closest obstacles that are in the path of the Robot to the goal T, and swap them with the closest holes in the branch of Ws (behind Ws).
    #               A.2.1.1) For each obstacle O in the path P from R to Ws (including R and Ws), pick the nearest hole H ahead of R and outside P and move it to v(O), with cost distance(v(O),v(H))
    #               A.2.1.2) Move R to Ws with cost distance(v(R),Ws)
    #               A.2.1.3) Solve the new instance of the problem using (B.1).
    #           A.2.2) NO: Can't reach sidestep vertex forwards. Then we try to move backwards. At this point the problem still might be possible, or not.
    #               A.2.2.1) Find the closest unvisited branch vertex Zb behind the robot R.
    #               A.2.2.2) Check If |Hb| >= distance(v(R), Zb) + 1.
    #               A.2.2.2.1) NO: Can't reach sidestep vertex backwards either. Problem is impossible.
    #               A.2.2.2.2) YES: Problem still might be possible, or not. By moving backwards, we might increase Hm (the miminum number of needed holes). We will know later. 
    #               A.2.2.2.2.1) For each sidestep neighbor Zs of Zb (away from the robot), see how many holes Hn are behind Zs (counting Zs as well)
    #               A.2.2.2.2.2) Pick Zs with minimal Hn
    #               A.2.2.2.2.3) For each obstacle O in the path P from R to Zs (including R and Zs), pick the nearest hole H behind of R and outside P and move it to v(O), with cost distance(v(O),v(H))
    #               A.2.2.2.2.4) Move R to Zs, with cost distance(v(R), Zs).
    #               A.2.2.2.2.5) Go back to (A.1). We now have a reachable branch vertex, so we know we'll go into (A.2.1) this time.
    # 
    # YES -> Solve it:
    # B.1) Calculate Hf -> number of holes available in front of the robot, Hm -> minimum number of holes available.
    # B.2) Check if distance(v(R),T) <= Hf.
    # B.2.1) YES: Let's go to the goal.
    # B.2.1.1) Fill path from R to T with the nearest holes. 
    # B.2.1.2) Move R to T. FINISHED!
    # B.2.2) NO: Let's go to the furthest reachable sidestep vertex.
    # B.2.2.1) Find the furthest reachable unvisited branch vertex Vb between the robot R and the goal T.
    # B.2.2.2) Say Vs is a random empty sidestep vertex neighbor to Vb, or a full one if none are free. 
    # B.2.2.3) For each obstacle O in the path P from R to Vs (including R and Vs), pick the nearest hole H ahead of R and outside P and move it to v(O), with cost distance(v(O),v(H))
    # B.2.2.4) Move the robot to Vs with cost distance(v(R),Vs).
    # B.2.2.5) Mark Vb as visited.
    # B.2.2.6) Go to (B.1)

# Helper method to generate rememberable names for the problem instances. Still-Characterize-Foot is way better than instance_001!
def getRandomName():
    words = ['school','state','never','become','between','high','really','something','most','another','much','family','own','out','leave','put','old','while','mean','on','keep','student','why','let','great','same','big','group','begin','seem','country','help','talk','where','turn','problem','every','start','hand','might','American','show','part','about','against','place','over','such','again','few','case','most','week','company','where','system','each','right','program','hear','so','question','during','work','play','government','run','small','number','off','always','move','like','night','live','Mr','point','believe','hold','today','bring','happen','next','without','before','large','all','million','must','home','under','water','room','write','mother','area','national','money','story','young','fact','month','different','lot','right','study','book','eye','job','word','though','business','issue','side','kind','four','head','far','black','long','both','little','house','yes','after','since','long','provide','service','around','friend','important','father','sit','away','until','power','hour','game','often','yet','line','political','end','among','ever','stand','bad','lose','however','member','pay','law','meet','car','city','almost','include','continue','set','later','community','much','name','five','once','white','least','president','learn','real','change','team','minute','best','several','idea','kid','body','information','nothing','ago','right','lead','social','understand','whether','back','watch','together','follow','around','parent','only','stop','face','anything','create','public','already','speak','others','read','level','allow','add','office','spend','door','health','person','art','sure','such','war','history','party','within','grow','result','open','change','morning','walk','reason','low','win','research','girl','guy','early','food','before','moment','himself','air','teacher','force','offer','enough','both','education','across','although','remember','foot','second','boy','maybe','toward','able','age','off','policy','everything','love','process','music','including','consider','appear','actually','buy','probably','human','wait','serve','market','die','send','expect','home','sense','build','stay','fall','oh','nation','plan','cut','college','interest','death','course','someone','experience','behind','reach','local','kill','six','remain','effect','use','yeah','suggest','class','control','raise','care','perhaps','little','late','hard','field','else','pass','former','sell','major','sometimes','require','along','development','themselves','report','role','better','economic','effort','up','decide','rate','strong','possible','heart','drug','show','leader','light','voice','wife','whole','police','mind','finally','pull','return','free','military','price','report','less','according','decision','explain','son','hope','even','develop','view','relationship','carry','town','road','drive','arm','TRUE','federal','break','better','difference','thank','receive','value','international','building','action','full','model','join','season','society','because','tax','director','early','position','player','agree','especially','record','pick','wear','paper','special','space','ground','form','support','event','official','whose','matter','everyone','center','couple','site','end','project','hit','base','activity','star','table','need','court','produce','eat','American','teach','oil','half','situation','easy','cost','industry','figure','face','street','image','itself','phone','either','data','cover','quite','picture','clear','practice','piece','land','recent','describe','product','doctor','wall','patient','worker','news','test','movie','certain','north','love','personal','open','support','simply','third','technology','catch','step','baby','computer','type','attention','draw','film','Republican','tree','source','red','nearly','organization','choose','cause','hair','look','point','century','evidence','window','difficult','listen','soon','culture','billion','chance','brother','energy','period','course','summer','less','realize','hundred','available','plant','likely','opportunity','term','short','letter','condition','choice','place','single','rule','daughter','administration','south','husband','Congress','floor','campaign','material','population','well','call','economy','medical','hospital','church','close','thousand','risk','current','fire','future','wrong','involve','defense','anyone','increase','security','bank','myself','certainly','west','sport','board','seek','per','subject','officer','private','rest','behavior','deal','performance','fight','throw','top','quickly','past','goal','second','bed','order','author','fill','represent','focus','foreign','drop','plan','blood','upon','agency','push','nature','color','no','recently','store','reduce','sound','note','fine','before','near','movement','page','enter','share','than','common','poor','other','natural','race','concern','series','significant','similar','hot','language','each','usually','response','dead','rise','animal','factor','decade','article','shoot','east','save','seven','artist','away','scene','stock','career','despite','central','eight','thus','treatment','beyond','happy','exactly','protect','approach','lie','size','dog','fund','serious','occur','media','ready','sign','thought','list','individual','simple','quality','pressure','accept','answer','hard','resource','identify','left','meeting','determine','prepare','disease','whatever','success','argue','cup','particularly','amount','ability','staff','recognize','indicate','character','growth','loss','degree','wonder','attack','herself','region','television','box','TV','training','pretty','trade','deal','election','everybody','physical','lay','general','feeling','standard','bill','message','fail','outside','arrive','analysis','benefit','name','forward','lawyer','present','section','environmental','glass','answer','skill','sister','PM','professor','operation','financial','crime','stage','ok','compare','authority','miss','design','sort','one','act','ten','knowledge','gun','station','blue','state','strategy','little','clearly','discuss','indeed','force','truth','song','example','democratic','check','environment','leg','dark','public','various','rather','laugh','guess','executive','set','study','prove','hang','entire','rock','design','enough','forget','since','claim','note','remove','manager','help','close','sound','enjoy','network','legal','religious','cold','form','final','main','science','green','memory','card','above','seat','cell','establish','nice','trial','expert','that','spring','firm','Democrat','radio','visit','management','care','avoid','imagine','tonight','huge','ball','no','close','finish','yourself','talk','theory','impact','respond','statement','maintain','charge','popular','traditional','onto','reveal','direction','weapon','employee','cultural','contain','peace','head','control','base','pain','apply','play','measure','wide','shake','fly','interview','manage','chair','fish','particular','camera','structure','politics','perform','bit','weight','suddenly','discover','candidate','top','production','treat','trip','evening','affect','inside','conference','unit','best','style','adult','worry','range','mention','rather','far','deep','past','edge','individual','specific','writer','trouble','necessary','throughout','challenge','fear','shoulder','institution','middle','sea','dream','bar','beautiful','property','instead','improve','stuff','detail','method','sign','somebody','magazine','hotel','soldier','reflect','heavy','cause','bag','heat','fall','marriage','tough','sing','surface','purpose','exist','pattern','whom','skin','agent','owner','machine','gas','down','ahead','generation','commercial','address','cancer','test','item','reality','coach','step','Mrs','yard','beat','violence','total','tend','investment','discussion','finger','garden','notice','collection','modern','task','partner','positive','civil','kitchen','consumer','shot','budget','wish','painting','scientist','safe','agreement','capital','mouth','nor','victim','newspaper','instead','threat','responsibility','smile','attorney','score','account','interesting','break','audience','rich','dinner','figure','vote','western','relate','travel','debate','prevent','citizen','majority','none','front','born','admit','senior','assume','wind','key','professional','mission','fast','alone','customer','suffer','speech','successful','option','participant','southern','fresh','eventually','no','forest','video','global','Senate','reform','access','restaurant','judge','publish','cost','relation','like','release','own','bird','opinion','credit','critical','corner','concerned','recall','version','stare','safety','effective','neighborhood','original','act','troop','income','directly','hurt','species','immediately','track','basic','strike','hope','sky','freedom','absolutely','plane','nobody','achieve','object','attitude','labor','refer','concept','client','powerful','perfect','nine','therefore','conduct','announce','conversation','examine','touch','please','attend','completely','vote','variety','sleep','turn','involved','investigation','nuclear','researcher','press','conflict','spirit','experience','replace','British','encourage','argument','by','once','camp','brain','feature','afternoon','AM','weekend','dozen','possibility','along','insurance','department','battle','beginning','date','generally','African','very','sorry','crisis','complete','fan','stick','define','easily','through','hole','element','vision','status','normal','Chinese','ship','solution','stone','slowly','scale','bit','university','introduce','driver','attempt','park','spot','lack','ice','boat','drink','sun','front','distance','wood','handle','truck','return','mountain','survey','supposed','tradition','winter','village','Soviet','refuse','sales','roll','communication','run','screen','gain','resident','hide','gold','club','future','farm','potential','increase','middle','European','presence','independent','district','shape','reader','Ms','contract','crowd','Christian','express','apartment','willing','strength','previous','band','obviously','horse','interested','target','prison','ride','guard','terms','demand','reporter','deliver','text','share','tool','wild','vehicle','observe','flight','inside','facility','understanding','average','emerge','advantage','quick','light','leadership','earn','pound','basis','bright','operate','guest','sample','contribute','tiny','block','protection','settle','feed','collect','additional','while','highly','identity','title','mostly','lesson','faith','river','promote','living','present','count','unless','marry','tomorrow','technique','path','ear','shop','folk','order','principle','survive','lift','border','competition','jump','gather','limit','fit','claim','cry','equipment','worth','associate','critic','warm','aspect','result','insist','failure','annual','French','Christmas','comment','responsible','affair','approach','until','procedure','regular','spread','chairman','baseball','soft','ignore','egg','measure','belief','demonstrate','anybody','murder','gift','religion','review','editor','past','engage','coffee','document','speed','cross','influence','anyway','threaten','commit','female','youth','wave','move','afraid','quarter','background','native','broad','wonderful','deny','apparently','slightly','reaction','twice','suit','perspective','growing','blow','construction','kind','intelligence','destroy','cook','connection','charge','burn','shoe','view','grade','context','committee','hey','mistake','focus','smile','location','clothes','Indian','quiet','dress','promise','aware','neighbor','complete','drive','function','bone','active','extend','chief','average','combine','wine','below','cool','voter','mean','demand','learning','bus','hell','dangerous','remind','moral','United','category','relatively','victory','key','academic','visit','Internet','healthy','fire','negative','following','historical','medicine','tour','depend','photo','finding','grab','direct','classroom','contact','justice','participate','daily','fair','pair','famous','exercise','knee','flower','tape','hire','familiar','appropriate','supply','fully','cut','will','actor','birth','search','tie','democracy','eastern','primary','yesterday','circle','device','progress','next','front','bottom','island','exchange','clean','studio','train','lady','colleague','application','neck','lean','damage','plastic','tall','plate','hate','otherwise','writing','press','male','start','alive','expression','football','intend','attack','chicken','army','abuse','theater','shut','map','extra','session','danger','welcome','domestic','lots','literature','rain','desire','assessment','injury','respect','northern','nod','paint','fuel','leaf','direct','dry','Russian','instruction','fight','pool','climb','sweet','lead','engine','fourth','salt','expand','importance','metal','fat','ticket','software','disappear','corporate','strange','lip','reading','urban','mental','increasingly','lunch','educational','somewhere','farmer','above','sugar','planet','favorite','explore','obtain','enemy','greatest','complex','surround','athlete','invite','repeat','carefully','soul','scientific','impossible','panel','meaning','mom','married','alone','instrument','predict','weather','presidential','emotional','commitment','Supreme','bear','pocket','thin','temperature','surprise','poll','proposal','consequence','half','breath','sight','cover','balance','adopt','minority','straight','attempt','connect','works','teaching','belong','aid','advice','okay','photograph','empty','regional','trail','novel','code','somehow','organize','jury','breast','Iraqi','human','acknowledge','theme','storm','union','record','desk','fear','thanks','fruit','under','expensive','yellow','conclusion','prime','shadow','struggle','conclude','analyst','dance','limit','like','regulation','being','last','ring','largely','shift','revenue','mark','locate','county','appearance','package','difficulty','bridge','recommend','obvious','train','basically','generate','anymore','propose','thinking','possibly','trend','visitor','loan','currently','comfortable','investor','but','profit','angry','crew','deep','accident','male','meal','hearing','traffic','muscle','notion','capture','prefer','truly','earth','Japanese','chest','search','thick','cash','museum','beauty','emergency','unique','feature','internal','ethnic','link','stress','content','select','root','nose','declare','outside','appreciate','actual','bottle','hardly','setting','launch','dress','file','sick','outcome','ad','defend','matter','judge','duty','sheet','ought','ensure','Catholic','extremely','extent','component','mix','slow','contrast','zone','wake','challenge','airport','chief','brown','standard','shirt','pilot','warn','ultimately','cat','contribution','capacity','ourselves','estate','guide','circumstance','snow','English','politician','steal','pursue','slip','percentage','meat','funny','neither','soil','influence','surgery','correct','Jewish','blame','estimate','due','basketball','late','golf','investigate','crazy','significantly','chain','address','branch','combination','just','frequently','governor','relief','user','dad','kick','part','manner','ancient','silence','rating','golden','motion','German','gender','solve','fee','landscape','used','bowl','equal','long','official','forth','frame','typical','except','conservative','eliminate','host','hall','trust','ocean','score','row','producer','afford','meanwhile','regime','division','confirm','fix','appeal','mirror','tooth','smart','length','entirely','rely','topic','complain','issue','variable','back','range','telephone','perception','attract','confidence','bedroom','secret','debt','rare','his','tank','nurse','coverage','opposition','aside','anywhere','bond','file','pleasure','master','era','requirement','check','stand','fun','expectation','wing','separate','now','clear','struggle','mean','somewhat','pour','stir','judgment','clean','except','beer','English','reference','tear','doubt','grant','seriously','account','minister','totally','hero','industrial','cloud','stretch','winner','volume','travel','seed','surprised','rest','fashion','pepper','separate','busy','intervention','copy','tip','below','cheap','aim','cite','welfare','vegetable','gray','dish','beach','improvement','everywhere','opening','overall','divide','initial','terrible','oppose','contemporary','route','multiple','essential','question','league','criminal','careful','core','upper','rush','necessarily','specifically','tired','rise','tie','employ','holiday','dance','vast','resolution','household','fewer','abortion','apart','witness','match','barely','sector','representative','lack','beneath','beside','black','incident','limited','proud','flow','faculty','increased','waste','merely','mass','emphasize','experiment','definitely','bomb','enormous','tone','liberal','massive','engineer','wheel','female','decline','invest','promise','cable','towards','expose','rural','AIDS','Jew','narrow','cream','secretary','gate','solid','hill','typically','noise','grass','unfortunately','hat','legislation','succeed','either','celebrate','achievement','fishing','drink','accuse','hand','useful','land','secret','reject','talent','taste','characteristic','milk','escape','cast','sentence','unusual','closely','convince','height','physician','assess','sleep','plenty','ride','virtually','first','addition','sharp','creative','lower','behind','approve','explanation','outside','gay','campus','proper','live','guilty','living','acquire','compete','technical','plus','mind','potential','immigrant','weak','illegal','hi','alternative','interaction','column','personality','signal','curriculum','list','honor','passenger','assistance','forever','fun','regard','Israeli','association','twenty','knock','review','wrap','lab','offer','display','criticism','asset','depression','spiritual','musical','journalist','prayer','suspect','scholar','warning','climate','cheese','observation','childhood','payment','sir','permit','cigarette','definition','priority','bread','creation','graduate','request','emotion','scream','dramatic','universe','gap','excellent','deeply','prosecutor','mark','green','lucky','drag','airline','library','agenda','recover','factory','selection','primarily','roof','unable','expense','initiative','diet','arrest','funding','therapy','wash','schedule','sad','brief','housing','post','purchase','existing','dark','steel','regarding','shout','remaining','visual','fairly','chip','violent','silent','suppose','self','bike','tea','perceive','comparison','settlement','layer','planning','far','description','later','slow','slide','widely','wedding','inform','portion','territory','immediate','opponent','abandon','link','mass','lake','transform','tension','display','leading','bother','consist','alcohol','enable','bend','saving','gain','desert','shall','error','release','cop','Arab','double','walk','sand','Spanish','rule','hit','preserve','passage','formal','transition','existence','album','participation','arrange','atmosphere','joint','reply','cycle','opposite','lock','whole','deserve','consistent','resistance','discovery','tear','exposure','pose','stream','sale','trust','benefit','pot','grand','mine','hello','coalition','tale','knife','resolve','racial','phase','present','joke','coat','Mexican','symptom','contact','manufacturer','philosophy','potato','interview','foundation','quote','online','pass','negotiation','good','urge','occasion','dust','breathe','elect','investigator','jacket','glad','ordinary','reduction','rarely','shift','pack','suicide','numerous','touch','substance','discipline','elsewhere','iron','practical','moreover','passion','volunteer','implement','essentially','gene','enforcement','vs','sauce','independence','marketing','priest','amazing','intense','advance','employer','shock','inspire','adjust','retire','sure','visible','kiss','illness','cap','habit','competitive','juice','congressional','involvement','dominate','previously','whenever','transfer','analyze','another','attach','for','Indian','disaster','parking','prospect','boss','complaint','championship','coach','exercise','fundamental','severe','enhance','mystery','impose','poverty','other','entry','fat','spending','king','evaluate','symbol','still','trade','maker','mood','accomplish','emphasis','illustrate','boot','monitor','Asian','entertainment','bean','evaluation','creature','commander','digital','arrangement','concentrate','total','usual','anger','psychological','heavily','peak','approximately','increasing','disorder','missile','equally','vary','wire','round','distribution','transportation','holy','ring','twin','command','commission','interpretation','breakfast','stop','strongly','engineering','luck','constant','race','clinic','veteran','smell','tablespoon','capable','nervous','tourist','light','toss','crucial','bury','pray','tomato','exception','butter','deficit','bathroom','objective','block','electronic','ally','journey','reputation','mixture','surely','tower','smoke','confront','pure','glance','dimension','toy','prisoner','fellow','smooth','nearby','peer','designer','personnel','shape','educator','relative','immigration','belt','teaspoon','birthday','implication','perfectly','coast','supporter','accompany','silver','teenager','recognition','retirement','flag','recovery','whisper','watch','gentleman','corn','moon','inner','junior','rather','throat','salary','swing','observer','due','straight','publication','pretty','crop','dig','strike','permanent','plant','phenomenon','anxiety','unlike','wet','literally','resist','convention','embrace','supply','assist','exhibition','construct','viewer','pan','consultant','soon','line','administrator','date','occasionally','mayor','consideration','CEO','secure','pink','smoke','estimate','buck','historic','poem','grandmother','bind','fifth','constantly','enterprise','favor','testing','stomach','apparent','weigh','install','sensitive','suggestion','mail','recipe','reasonable','preparation','wooden','elementary','concert','aggressive','FALSE','intention','channel','extreme','tube','drawing','protein','quit','absence','roll','Latin','rapidly','jail','comment','diversity','honest','Palestinian','pace','employment','speaker','impression','essay','respondent','giant','cake','historian','negotiate','restore','substantial','pop','particular','specialist','origin','approval','mine','quietly','advise','conventional','drop','count','depth','wealth','disability','shell','general','criticize','fast','professional','effectively','biological','pack','onion','deputy','flat','brand','assure','mad','award','criteria','dealer','via','alternative','utility','precisely','arise','armed','nevertheless','highway','clinical','routine','schedule','wage','normally','phrase','ingredient','stake','Muslim','dream','fiber','activist','Islamic','snap','terrorism','refugee','incorporate','hip','ultimate','switch','corporation','valuable','assumption','gear','graduate','barrier','minor','provision','killer','assign','gang','developing','classic','chemical','wave','label','teen','index','vacation','advocate','draft','extraordinary','heaven','rough','yell','pregnant','distant','drama','satellite','personally','wonder','clock','chocolate','Italian','Canadian','ceiling','sweep','advertising','universal','spin','house','button','bell','rank','darkness','ahead','clothing','super','yield','fence','portrait','paint','survival','roughly','lawsuit','bottom','testimony','bunch','beat','wind','found','burden','react','chamber','furniture','cooperation','string','ceremony','communicate','taste','cheek','lost','profile','mechanism','disagree','like','penalty','match','ie','advance','resort','destruction','bear','unlikely','tissue','constitutional','pant','stranger','infection','cabinet','broken','apple','electric','proceed','track','bet','literary','virus','stupid','dispute','fortune','strategic','assistant','overcome','remarkable','occupy','statistics','shopping','cousin','encounter','wipe','initially','blind','white','port','honor','electricity','genetic','adviser','pay','spokesman','retain','latter','incentive','slave','chemical','translate','accurate','whereas','terror','though','expansion','elite','Olympic','dirt','odd','rice','bullet','tight','Bible','chart','solar','decline','conservative','process','square','stick','concentration','complicated','gently','champion','scenario','telescope','reflection','revolution','strip','interpret','friendly','tournament','fiction','detect','balance','likely','tremendous','lifetime','recommendation','flow','senator','market','hunting','salad','guarantee','innocent','boundary','pause','remote','satisfaction','journal','bench','lover','raw','awareness','surprising','withdraw','general','deck','similarly','newly','pole','testify','mode','dialogue','imply','naturally','mutual','founder','top','advanced','pride','dismiss','aircraft','delivery','mainly','bake','freeze','platform','finance','sink','attractive','respect','diverse','relevant','ideal','joy','worth','regularly','working','singer','evolve','shooting','partly','unknown','assistant','offense','counter','DNA','smell','potentially','transfer','thirty','justify','protest','crash','craft','treaty','terrorist','insight','possess','politically','tap','lie','extensive','episode','double','swim','tire','fault','loose','free','shortly','originally','considerable','prior','intellectual','mix','assault','relax','stair','adventure','external','proof','confident','headquarters','sudden','dirty','violation','tongue','license','hold','shelter','rub','controversy','entrance','favorite','practice','properly','fade','defensive','tragedy','net','characterize','funeral','profession','alter','spot','constitute','establishment','squeeze','imagination','target','mask','convert','comprehensive','prominent','presentation','regardless','easy','load','stable','introduction','appeal','pretend','not','elderly','representation','deer','split','violate','partnership','pollution','emission','steady','vital','neither','fate','earnings','oven','distinction','segment','nowhere','poet','mere','exciting','variation','comfort','radical','stress','adapt','Irish','honey','correspondent','pale','musician','significance','load','round','vessel','storage','flee','leather','distribute','evolution','ill','tribe','shelf','can','grandfather','lawn','buyer','dining','wisdom','council','vulnerable','instance','garlic','capability','poetry','celebrity','gradually','stability','doubt','fantasy','scared','guide','plot','framework','gesture','depending','ongoing','psychology','since','counselor','witness','chapter','fellow','divorce','owe','pipe','athletic','slight','math','shade','tail','sustain','mount','obligation','angle','palm','differ','custom','store','economist','fifteen','soup','celebration','efficient','damage','composition','satisfy','pile','briefly','carbon','closer','consume','scheme','crack','frequency','tobacco','survivor','besides','in','psychologist','wealthy','galaxy','given','fund','ski','limitation','OK','trace','appointment','preference','meter','explosion','arrest','publicly','incredible','fighter','rapid','admission','hunter','educate','painful','friendship','aide','infant','calculate','fifty','rid','porch','tendency','uniform','formation','scholarship','reservation','efficiency','waste','qualify','mall','derive','scandal','PC','helpful','impress','heel','resemble','privacy','fabric','surprise','contest','proportion','guideline','rifle','maintenance','conviction','trick','organic','tent','examination','publisher','strengthen','French','proposed','myth','sophisticated','cow','etc','standing','asleep','tennis','nerve','barrel','bombing','membership','ratio','menu','purchase','controversial','desperate','rate','lifestyle','humor','loud','glove','suspect','sufficient','narrative','photographer','helicopter','Catholic','modest','provider','delay','agricultural','explode','stroke','scope','punishment','handful','badly','horizon','curious','downtown','girlfriend','prompt','request','cholesterol','absorb','adjustment','taxpayer','eager','principal','detailed','motivation','assignment','restriction','across','Palestinian','laboratory','workshop','differently','auto','romantic','cotton','motor','sue','flavor','overlook','float','undergo','sequence','demonstration','jet','orange','consumption','assert','blade','temporary','medication','print','cabin','bite','relative','edition','valley','yours','pitch','pine','brilliant','versus','manufacturing','risk','Christian','complex','absolute','chef','discrimination','offensive','German','suit','boom','register','appoint','heritage','God','terrorist','dominant','successfully','lemon','hungry','sense','dry','wander','submit','economics','naked','anticipate','nut','legacy','extension','shrug','fly','battery','arrival','legitimate','orientation','inflation','cope','flame','cluster','host','wound','dependent','shower','institutional','depict','operating','flesh','garage','operator','instructor','collapse','borrow','furthermore','comedy','mortgage','sanction','civilian','twelve','weekly','habitat','grain','brush','consciousness','devote','crack','measurement','province','ease','seize','ethics','nomination','permission','wise','actress','summit','acid','odds','gifted','frustration','medium','function','physically','grant','distinguish','shore','repeatedly','lung','firm','running','correct','distinct','artistic','discourse','basket','ah','fighting','impressive','competitor','ugly','worried','portray','powder','ghost','persuade','moderate','subsequent','continued','cookie','carrier','cooking','frequent','ban','swing','orange','awful','admire','pet','miracle','exceed','rhythm','widespread','killing','lovely','sin','charity','script','tactic','identification','transformation','everyday','headline','crash','venture','invasion','military','nonetheless','adequate','piano','grocery','intensity','exhibit','high','blanket','margin','principal','quarterback','mouse','rope','concrete','prescription','chase','document','brick','recruit','patch','consensus','horror','recording','changing','painter','colonial','pie','sake','gaze','courage','pregnancy','swear','defeat','clue','reinforce','win','confusion','slice','occupation','dear','coal','sacred','criminal','formula','cognitive','collective','exact','uncle','square','captain','sigh','attribute','dare','okay','homeless','cool','gallery','soccer','defendant','tunnel','fitness','lap','grave','toe','container','virtue','abroad','architect','dramatically','makeup','inquiry','rose','surprisingly','highlight','decrease','indication','rail','anniversary','couch','alliance','hypothesis','boyfriend','compose','peer','mess','rank','legend','regulate','adolescent','shine','norm','upset','remark','resign','reward','gentle','related','organ','lightly','concerning','invent','laughter','fit','northwest','counseling','tight','receiver','ritual','insect','interrupt','salmon','favor','trading','concern','magic','superior','combat','stem','surgeon','acceptable','physics','rape','counsel','brush','jeans','hunt','continuous','log','echo','pill','excited','sculpture','compound','integrate','flour','bitter','bare','slope','rent','presidency','serving','subtle','greatly','bishop','drinking','delay','cry','acceptance','collapse','shop','pump','candy','evil','final','finance','pleased','medal','beg','sponsor','ethical','secondary','slam','export','experimental','melt','midnight','net','curve','integrity','entitle','evident','logic','essence','park','exclude','harsh','closet','suburban','greet','favor','interior','corridor','murder','retail','pitcher','march','snake','pitch','excuse','cross','weakness','pig','cold','classical','estimated','online','unemployment','civilization','fold','patient','pop','daily','reverse','missing','correlation','humanity','flash','developer','reliable','excitement','beef','Islam','Roman','stretch','architecture','occasional','administrative','elbow','deadly','Muslim','Hispanic','allegation','tip','confuse','airplane','monthly','duck','dose','Korean','plead','initiate','lecture','van','sixth','bay','mainstream','suburb','sandwich','unlike','trunk','rumor','implementation','swallow','motivate','render','longtime','trap','restrict','cloth','seemingly','legislative','effectiveness','enforce','lens','reach','inspector','lend','plain','fraud','companion','contend','nail','array','strict','assemble','frankly','rat','burst','hallway','cave','inevitable','southwest','monster','speed','protest','unexpected','obstacle','facilitate','encounter','rip','herb','overwhelming','integration','crystal','recession','wish','top','written','motive','label','flood','pen','ownership','nightmare','notice','inspection','supervisor','consult','arena','laugh','diagnosis','possession','forgive','warm','consistently','basement','project','drift','drain','last','prosecution','maximum','announcement','warrior','prediction','bacteria','questionnaire','mud','infrastructure','hurry','privilege','temple','medium','outdoor','suck','and/or','broadcast','re','leap','random','past','wrist','curtain','monitor','pond','domain','guilt','cattle','subject','walking','playoff','minimum','fiscal','skirt','dump','hence','database','uncomfortable','aim','execute','limb','ideology','average','welcome','tune','continuing','harm','railroad','endure','radiation','horn','chronic','peaceful','innovation','strain','guitar','replacement','behave','administer','simultaneously','dancer','amendment','guard','pad','transmission','await','retired','trigger','spill','grateful','grace','virtual','response','colony','adoption','slide','indigenous','closed','convict','civilian','towel','modify','particle','award','glance','prize','landing','conduct','blue','boost','bat','alarm','festival','grip','weird','undermine','freshman','sweat','outer','European','drunk','survey','research','separation','traditionally','stuff','govern','southeast','intelligent','wherever','ballot','rhetoric','convinced','driving','vitamin','enthusiasm','accommodate','praise','injure','wilderness','nearby','endless','mandate','pause','excuse','respectively','uncertainty','chaos','short','mechanical','canvas','forty','matter','lobby','profound','format','trait','currency','turkey','reserve','beam','abuse','astronomer','corruption','contractor','apologize','doctrine','genuine','thumb','unity','compromise','horrible','behavioral','exclusive','scatter','commonly','convey','rush','twist','complexity','fork','disk','relieve','suspicion','lock','finish','residence','shame','meaningful','sidewalk','Olympics','technological','signature','pleasant','wow','suspend','rebel','frozen','desire','spouse','fluid','pension','resume','theoretical','sodium','blow','promotion','delicate','forehead','rebuild','bounce','electrical','hook','detective','traveler','click','compensation','signal','exit','attraction','dedicate','altogether','pickup','carve','needle','belly','ship','scare','portfolio','shuttle','invisible','timing','engagement','ankle','transaction','rescue','counterpart','historically','firmly','mild','rider','doll','noon','exhibit','amid','identical','precise','anxious','structural','residential','loud','diagnose','carbohydrate','liberty','poster','theology','nonprofit','crawl','oxygen','handsome','magic','sum','provided','businessman','promising','conscious','determination','donor','hers','pastor','jazz','opera','Japanese','bite','frame','evil','acquisition','pit','hug','wildlife','punish','giant','primary','equity','wrong','doorway','departure','elevator','teenage','guidance','happiness','statue','pursuit','repair','decent','gym','oral','clerk','Israeli','envelope','reporting','destination','fist','endorse','exploration','generous','bath','rescue','thereby','overall','indicator','sunlight','feedback','spectrum','purple','laser','bold','reluctant','starting','expertise','practically','program','picture','tune','eating','age','volunteer','hint','sharply','parade','advocate','realm','ban','strip','cancel','blend','therapist','slice','peel','pizza','recipient','hesitate','flip','accounting','debate','bias','huh','metaphor','candle','handle','worry','judicial','entity','suffering','feel','lamp','garbage','servant','addition','regulatory','diplomatic','elegant','inside','reception','vanish','automatically','hay','trail','necessity','confess','racism','starter','interior','banking','casual','gravity','enroll','diminish','prevention','Arab','value','minimize','chop','performer','intent','isolate','pump','inventory','productive','assembly','civic','silk','magnitude','steep','hostage','collector','popularity','kiss','alien','dynamic','scary','equation','angel','switch','offering','rage','photography','repair','toilet','disappointed','precious','prohibit','representative','content','realistic','Russian','hidden','command','tender','wake','gathering','outstanding','stumble','lonely','automobile','artificial','dawn','abstract','descend','silly','hook','tide','shared','hopefully','readily','cooperate','revolutionary','romance','hardware','pillow','kit','cook','spread','continent','seal','circuit','sink','ruling','shortage','annually','lately','trap','scan','fool','deadline','rear','processing','ranch','coastal','undertake','softly','reserve','burning','verbal','tribal','ridiculous','automatic','diamond','credibility','import','spring','way','divine','sentiment','cart','oversee','stem','elder','pro','inspiration','Dutch','quantity','trailer','mate','Greek','genius','monument','bid','quest','sacrifice','invitation','accuracy','juror','officially','broker','treasure','loyalty','credit','shock','talented','gasoline','stiff','output','nominee','extended','please','diabetes','slap','toxic','alleged','jaw','grief','mysterious','rocket','donate','inmate','tackle','dynamics','bow','ours','senior','dignity','carpet','parental','bubble','heat','buddy','barn','sword','flash','seventh','glory','tightly','protective','tuck','drum','faint','post','queen','dilemma','input','specialize','northeast','shallow','liability','sail','merchant','stadium','improved','bloody','defeat','associated','withdrawal','refrigerator','nest','near','thoroughly','lane','ancestor','condemn','steam','accent','escape','optimistic','unite','cage','equip','shrimp','homeland','exchange','rack','costume','wolf','courtroom','statute','cartoon','besides','productivity','grin','symbolic','seal','bug','bless','aunt','agriculture','rock','hostile','root','conceive','combined','instantly','bankruptcy','vaccine','bonus','collaboration','mixed','opposed','orbit','grasp','patience','spite','tropical','voting','patrol','willingness','position','revelation','rent','calm','jewelry','Cuban','haul','concede','trace','wagon','afterward','spectacular','ruin','sheer','prior','immune','reliability','ass','alongside','bush','exotic','fascinating','secure','clip','thigh','bull','drawer','regard','sheep','discourage','coordinator','ideological','runner','secular','intimate','empire','cab','divorce','exam','documentary','neutral','biology','flexible','progressive','web','conspiracy','catch','casualty','republic','execution','terrific','whale','functional','star','draft','instinct','teammate','aluminum','whoever','ministry','verdict','instruct','skull','ease','cooperative','manipulate','bee','practitioner','loop','edit','whip','puzzle','mushroom','subsidy','boil','tragic','mathematics','mechanic','jar','respect','earthquake','pork','creativity','safely','underlying','dessert','sympathy','fisherman','incredibly','isolation','sock','near','jump','eleven','entrepreneur','syndrome','bureau','seat','workplace','ambition','touchdown','utilize','breeze','costly','ambitious','Christianity','presumably','influential','translation','uncertain','dissolve','object','statistical','gut','metropolitan','rolling','aesthetic','spell','insert','booth','helmet','waist','expected','lion','accomplishment','royal','panic','cast','crush','actively','cliff','minimal','cord','fortunately','cocaine','illusion','anonymous','tolerate','appreciation','commissioner','harm','flexibility','instructional','scramble','casino','tumor','decorate','sort','charge','pulse','equivalent','fixed','experienced','donation','diary','sibling','irony','spoon','midst','alley','upset','interact','soap','cute','rival','punch','pin','hockey','passing','persist','supplier','known','momentum','purse','shed','liquid','icon','elephant','consequently','legislature','associate','franchise','correctly','mentally','foster','bicycle','encouraging','cheat','access','heal','fever','filter','rabbit','coin','exploit','accessible','organism','sensation','partially','stay','upstairs','dried','minimum','pro','conservation','shove','backyard','charter','stove','consent','comprise','reminder','alike','placement','dough','grandchild','dam','reportedly','surrounding','ecological','outfit','unprecedented','columnist','workout','preliminary','patent','shy','quote','trash','disabled','gross','damn','hormone','texture','counter','pencil','associate','frontier','spray','bet','disclose','custody','banker','beast','interfere','oak','case','eighth','notebook','outline','gaze','attendance','speculation','uncover','behalf','innovative','shark','reward','mill','installation','stimulate','tag','vertical','swimming','fleet','catalog','outsider','sacrifice','desperately','stance','compel','sensitivity','someday','instant','debut','proclaim','worldwide','hike','required','confrontation','colorful','ideal','constitution','trainer','Thanksgiving','scent','stack','eyebrow','sack','cease','inherit','tray','pioneer','organizational','textbook','uh','nasty','shrink','model','emerging','dot','wheat','fierce','envision','rational','kingdom','aisle','weaken','protocol','exclusively','vocal','marketplace','openly','unfair','terrain','deploy','risky','pasta','genre','distract','merit','planner','depressed','chunk','closest','discount','no','ladder','jungle','migration','breathing','invade','hurricane','retailer','classify','wound','coup','aid','ambassador','density','supportive','curiosity','skip','aggression','stimulus','journalism','robot','flood','dip','likewise','informal','Persian','feather','sphere','tighten','boast','pat','perceived','sole','publicity','major','unfold','joke','validity','ecosystem','strictly','partial','collar','weed','compliance','streak','supposedly','added','builder','glimpse','premise','specialty','deem','artifact','sneak','monkey','mentor','listener','lightning','legally','sleeve','disappointment','disturb','rib','excessive','debris','pile','rod','logical','liberal','ash','socially','parish','slavery','blank','commodity','cure','mineral','hunger','dying','developmental','faster','spare','halfway','cure','equality','cemetery','harassment','deliberately','fame','regret','striking','likelihood','carrot','atop','toll','rim','embarrassed','cling','isolated','blink','suspicious','wheelchair','squad','eligible','processor','plunge','this','sponsor','grin','color','demographic','rain','chill','refuge','steer','legislator','rally','programming','cheer','outlet','intact','vendor','thrive','peanut','chew','elaborate','intellectual','conception','auction','steak','comply','triumph','shareholder','comparable','transport','conscience','calculation','considerably','interval','scratch','awake','jurisdiction','inevitably','feminist','constraint','emotionally','expedition','allegedly','compromise','strain','similarity','butt','lid','dumb','bulk','sprinkle','mortality','philosophical','conversion','patron','municipal','any','liver','harmony','solely','tolerance','instant','goat','arm','blessing','banana','running','palace','formerly','peasant','neat','grandparent','lawmaker','supermarket','cruise','mobile','plain','part','calendar','widow','deposit','beard','brake','downtown','screening','impulse','forbid','fur','brutal','predator','poke','opt','voluntary','trouble','valid','forum','dancing','happily','soar','removal','autonomy','enact','round','thread','light','landmark','unhappy','offender','coming','privately','fraction','distinctive','tourism','threshold','calm','routinely','suite','remark','regulator','straw','theological','apart','exhaust','globe','fragile','objection','chemistry','crowded','circle','blast','prevail','overnight','denial','rental','fantastic','fragment','level','screw','warmth','undergraduate','liquid','headache','policeman','yield','projection','battle','suitable','mention','graduation','drill','cruel','mansion','regard','grape','authorize','cottage','driveway','charm','loyal','clay','pound','balloon','invention','ego','fare','homework','disc','sofa','guarantee','availability','radar','frown','regain','leave','permit','sweater','rehabilitation','rubber','retreat','molecule','freely','favorable','steadily','veteran','integrated','ha','youngster','broadcast','premium','accountability','overwhelm','contemplate','update','spark','ironically','fatigue','beyond','speculate','marker','low','preach','bucket','bomb','blond','confession','provoke','marble','substantially','twist','defender','fish','explicit','transport','disturbing','surveillance','magnetic','technician','mutter','devastating','depart','arrow','trauma','neighboring','soak','ribbon','meantime','transmit','screen','harvest','consecutive','republican','coordinate','worldwide','within','spy','slot','riot','nutrient','citizenship','severely','sovereignty','ridge','brave','lighting','specify','contributor','frustrate','crowd','articulate','importantly','transit','dense','seminar','electronics','sunny','shorts','swell','accusation','soften','photograph','straighten','terribly','cue','sudden','bride','biography','hazard','compelling','seldom','tile','economically','honestly','troubled','bow','twentieth','balanced','foreigner','launch','convenience','delight','weave','timber','till','accurately','plea','bulb','copy','flying','sustainable','devil','bolt','cargo','spine','seller','skilled','managing','public','marine','dock','organized','fog','diplomat','boring','sometime','summary','missionary','epidemic','fatal','trim','warehouse','accelerate','butterfly','bronze','drown','inherent','praise','nationwide','spit','harvest','kneel','vacuum','selected','dictate','stereotype','sensor','laundry','manual','pistol','naval','plaintiff','kid','apology','till']

    first = words[random.randint(0,len(words) - 1)]
    second = words[random.randint(0,len(words) - 1)]
    third = words[random.randint(0,len(words) - 1)]
    return first + '-' + second + '-' + third

class Problem:
    # Generate a random graph. Note that it is possible to concatenate chains, so
    # it is possible (and likely) that maxChainLength will be exceeded.
    # Parameters:
    # - chainCount: amount of chains that compose the graph
    # - minChainLenght: lower bound for the chain lengths
    # - maxChainLength: upper bound for the chain lengths (before concatenations)
    # - cycleCount: how many cycles the graph should contain
    # - obstacleRatio: how many obstacles should be in the graph, in percentage relating to quantity of nodes
    def generateGraph(self, chainCount = 5, minChainLength = 1, maxChainLength = 3, cycleCount = 0, obstacleRatio = 0.7):
        # Initalize variables
        printStatus("Initializing...")
        graph = nx.Graph()
        chainEndings = set()
        currentIndex = 0

        printStatus("Creating chains")
        # Create chains
        for i in range(chainCount):
            if len(chainEndings) > 0:
                starter = random.choice(list(chainEndings))
            else:
                starter = currentIndex

            chainEndings.add(starter)    
            chainLength = random.randint(minChainLength,maxChainLength)

            for j in range(chainLength):
                graph.add_edge(starter,currentIndex + 1)
                currentIndex += 1
                starter = currentIndex 
            chainEndings.add(starter)
        
        # Create cycles
        for i in range(cycleCount):
            currentIndex += 1
            firstNode = random.choice(list(chainEndings))
            secondNode = random.choice(list(chainEndings))
            graph.add_edge(firstNode, currentIndex)
            graph.add_edge(currentIndex, secondNode)
        
        # Store graph
        self.graph = graph

        # Create robot
        nodesCopy = list(graph.nodes)
        random.shuffle(nodesCopy)
        self.robot = nodesCopy[0]
        nodesCopy.remove(self.robot)

        # Create obstacles
        obstacleCount = int(max(0, min(obstacleRatio, 1)) * len(graph.nodes))
        self.obstacles = nodesCopy[:obstacleCount]

        # Create goal
        goalIndex = random.randint(0,len(nodesCopy) - 1)
        self.goal = nodesCopy[goalIndex]

        # Create name
        self.name = str(len(list(self.graph.nodes))) + '-' + getRandomName()

    def initialize(self, savePics = False):
        # Reset total cost, moves, and the cached distance matrix
        self.totalCost = 0
        self.currentMove = 0
        self.moves = []
        self.savePicsOfMoves = savePics
        self.stateMap = {}
        nodeList = list(map(int,self.graph.nodes))
        self.cachedDistance = []
        self.branchVertexes = []
        i = 0
        for node in nodeList:
            self.stateMap[node] = HOLE_TAG
            if self.graph.degree[node] > 2:
                self.branchVertexes.append(node)
            if i > 0:
                self.cachedDistance.append([])
                for _ in range(i):
                    self.cachedDistance[i - 1].append(-1)
            i += 1
        for node in self.obstacles:
            self.stateMap[node] = OBSTACLE_TAG
        self.stateMap[self.robot] = ROBOT_TAG

        if self.savePicsOfMoves:
            self.drawGraph(savingImage=True,isMove=True)

    def drawGraph(self,savingImage=False,isMove=False):
        # Create labels
        labels = {}
        for node in self.graph.nodes:
            labels[node] = node

        # Draw resulting graph
        goalColor = '#55ff55'
        if self.goal in self.obstacles:
            goalColor = '#007700'
        elif self.goal == self.robot:
            goalColor = '#2277ff'

        if not hasattr(self, 'graphLayout'):
            self.graphLayout = nx.spring_layout(self.graph)
        pos = self.graphLayout

        nx.draw_networkx_nodes(self.graph,pos,node_color='#dddddd', node_size=200)
        nx.draw_networkx_nodes(self.graph,pos,nodelist=self.obstacles,node_color='#666666', node_size=200)
        nx.draw_networkx_nodes(self.graph,pos,nodelist=[self.robot],node_color='#dd77dd', node_size=200)
        nx.draw_networkx_nodes(self.graph,pos,nodelist=[self.goal],node_color=goalColor, node_size=200)
        nx.draw_networkx_edges(self.graph,pos)
        nx.draw_networkx_labels(self.graph,pos,labels,font_size=10)

        plt.title(self.name)
        imgPath = 'instances/'
        if isMove:
            imgPath = 'solutions/'
            plt.suptitle('Move: {0}     Current running cost: {1}'.format(self.currentMove, self.totalCost))
            imgPath += self.name + '_{0}.jpeg'.format(self.currentMove)
        else:
            imgPath += self.name + '.jpeg' 

        if savingImage:
            plt.savefig(imgPath,dpi=80)
        else:
            plt.show()
        plt.close()
    
    def read(self, problemDict):
        graph = nx.Graph()
        edgeDict = problemDict["edges"]
        for node, neighbors in edgeDict.items():
            for neighbor in neighbors:
                graph.add_edge(int(node), neighbor)
        self.graph = graph
        self.obstacles = problemDict["obstacles"]
        self.robot = problemDict["robot"]
        self.goal = problemDict["goal"]
        self.name = problemDict["name"]
        printStatus('Read instance: ' + self.name)
        # printStatus(problemDict)

    def export(self):
        problemDict = {}
        edges = {}
        for node in self.graph.nodes:
            neighbors = list(self.graph.neighbors(node))
            edges[int(node)] = [x for x in neighbors if x > node]
        problemDict["edges"] = edges
        problemDict["obstacles"] = self.obstacles
        problemDict["robot"] = self.robot
        problemDict["goal"] = self.goal
        problemDict["name"] = self.name

        with open('instances/' + self.name + '.txt',mode='w') as f:
            f.write(json.dumps(problemDict))
        self.drawGraph(savingImage=True)
        
        return problemDict

    def tagBranches(self):
        self.rootNode = self.robot
        self.parentDict = {self.robot: -1}
        self.childrenDict = {}
        for node in list(map(int,self.graph.nodes)):
            self.childrenDict[node] = []
        
        self.depth = {self.robot: 0}

        nodeTags = {self.robot:0}
        initialNeighbors = list(map(int,self.graph.neighbors(self.robot)))
        toVisit = initialNeighbors
        branchTag = 1
        for node in initialNeighbors:
            nodeTags[node] = branchTag
            self.parentDict[node] = self.robot
            self.childrenDict[self.robot].append(node)
            self.depth[node] = 1
            branchTag += 1
        
        while len(toVisit) > 0:
            currentNode = toVisit.pop()
            neighbors = list(map(int,self.graph.neighbors(currentNode)))
            for neighbor in neighbors:
                if neighbor in nodeTags:
                    continue
                if neighbor in initialNeighbors:
                    nodeTags[neighbor] = branchTag
                    branchTag += 1
                else:
                    nodeTags[neighbor] = nodeTags[currentNode]
                self.parentDict[neighbor] = currentNode
                self.depth[neighbor] = self.depth[self.parentDict[neighbor]] + 1
                self.childrenDict[currentNode].append(neighbor)
                
                toVisit.append(neighbor)
        
        goalTag = nodeTags[self.goal]
        holesFront = 0
        holesBehind = 0
        for key, value in nodeTags.items():
            if self.stateMap[key] == HOLE_TAG:
                if value == goalTag:
                    holesFront += 1
                else:
                    holesBehind+= 1
        
        self.nodeTags = nodeTags
        self.frontHoles = holesFront
        self.backHoles = holesBehind

    def findPathFromRobotToNode(self, target):
        currentNode = target
        path = []
        while currentNode != self.robot:
            path = [currentNode] + path
            currentNode = self.parentDict[currentNode]
        return path

    # This is O(N), but can be rewritten in O(log N). 
    # https://www.hackerrank.com/topics/lowest-common-ancestor for details
    def lowestCommonAncestor(self, currentNode, nodeA, nodeB):
        if currentNode == None:
            return None
        
        if currentNode == nodeA or currentNode == nodeB:
            return currentNode

        foundNodes = []
        for childNode in self.childrenDict[currentNode]:
            result = self.lowestCommonAncestor(childNode, nodeA, nodeB)
            if result != None:
                foundNodes.append(result)
    
        if len(foundNodes) == 2:
            return currentNode

        if len(foundNodes) == 1:
            return foundNodes[0]

        return None

    def distance(self, nodeA, nodeB):
        if nodeA == nodeB:
            return 0

        if nodeA < nodeB:
            aux = nodeA
            nodeA = nodeB
            nodeB = aux
        
        # At this point nodeA is greater than nodeB
        if self.cachedDistance[nodeA - 1][nodeB] > -1:
            return self.cachedDistance[nodeA - 1][nodeB]
        lca = self.lowestCommonAncestor(self.rootNode, nodeA, nodeB)
        distance = self.depth[nodeA] + self.depth[nodeB] - 2 * self.depth[lca]
        self.cachedDistance[nodeA - 1][nodeB] = distance

        return distance

    def updatePathAndMinimumHolesNeeded(self):
        path = self.findPathFromRobotToNode(self.goal)
        printStatus('Path from robot ('+ str(self.robot) +') to goal ('+str(self.goal)+'): ' + str(path))
        printStatus('Distance from robot to goal: ' + str(self.distance(self.robot, self.goal)))

        distance = 0
        minHoles = 0
        pathLength = len(path)
        for i in range(pathLength):
            distance += 1
            if path[i] in self.branchVertexes:
                printStatus('Found branch vertex ' + str(path[i]))
                if distance + 1 > minHoles:
                    minHoles = distance + 1
                    printStatus('Updating minHoles to ' + str(minHoles))
                distance = 1
            elif i == pathLength - 1:
                printStatus('Found goal vertex ' + str(path[i]))
                if distance > minHoles:
                    minHoles = distance
                    printStatus('Updating minHoles to ' + str(minHoles))

        printStatus('Minimum needed holes: ' + str(minHoles))
        self.minHoles = minHoles
        self.pathToGoal = path

    def tryToSolve(self):
        solutionFound = False
        impossibleInstance = False

        while not solutionFound and not impossibleInstance:
            # Check if we are at the goal
            if self.robot == self.goal:
                printStatus('SUCESS! Robot reached its goal. These were the moves we did:')
                printStatus(self.moves)
                printStatus('What did it cost? {0}'.format(self.totalCost))
                solutionFound = True
                break

            # Not at the goal. Update tags and variables
            self.tagBranches()
            self.updatePathAndMinimumHolesNeeded()
            
            printStatus('Trying to solve problem! Hm = {0}, Hf = {1}, Hb = {2}'.format(self.minHoles, self.frontHoles, self.backHoles))
            if self.frontHoles + self.backHoles < self.minHoles:
                printStatus('UNSOLVABLE.')
                impossibleInstance = True
            else:
                # Try to move forwards
                if self.tryToMoveForward():
                    printStatus('Moved forward!')
                else:
                    # Cannot move forwards. Try to move backwards.
                    printStatus('Cannot move forward! Trying to move backwards.')
                    if self.tryToMoveBackwards():
                        printStatus('Moved backwards!')
                    else:
                        # Can't move backwards. The problem is impossible.
                        printStatus('Cannot move backwards either :(')
                        impossibleInstance = True

        return solutionFound and not impossibleInstance

    def tryToMoveForward(self):
        distanceToGoal = len(self.pathToGoal)
        if self.frontHoles >= distanceToGoal:
            # We can reach the goal from here!
            # Clear path to goal
            self.clearPathFromRobotToNode(self.goal, self.pathToGoal)

            # Move robot to sidestepVertex
            self.moveRobotToNode(self.goal)

            return True
        else:
            furthestBranchVertex = self.getNearestSidestepVertexAhead()
            printStatus('Furthest Vb ahead: {0}'.format(furthestBranchVertex))
            if furthestBranchVertex == None:
                # Impossible to move forwards. 
                # Does not mean the problem is unsolvable, as we might be able to move backwards.
                return False
            else:
                # Can move forward! This means the problem is solvable.
                chosenSidestep = self.findBestSidestepVertexOfBranchVertex(furthestBranchVertex, nodesToAvoid=self.pathToGoal)
                
                # Find path from robot to sidestepVertex
                pathToSidestep = self.findPathFromRobotToNode(chosenSidestep)

                # Clear path from robot to sidestepVertex
                self.clearPathFromRobotToNode(chosenSidestep, pathToSidestep)

                # We may need the holes behind the sidestep vertex, so get them now
                self.fillHolesOfSidestepBranchIfNeeded(furthestBranchVertex, chosenSidestep)

                # Move robot to sidestepVertex
                self.moveRobotToNode(chosenSidestep)

                return True

    def tryToMoveBackwards(self):
        # Get nearest branch vertex backwards
        nearestBranchVertex = self.getNearestSidestepVertexBehind()
        printStatus('Nearest branch vertex behind the robot:{0}'.format(nearestBranchVertex))
        if nearestBranchVertex == None:
            return False

        # Check if we can reach it, then do a sidestep
        neededHoles = self.distance(self.robot, nearestBranchVertex) + 1
        if self.backHoles < neededHoles:
            return False

        # Can move backwards! This does not mean the problem is solvable. 
        # But we can transform it in a new problem, with no backwards step.
        # First we pick a sidestep vertex
        nodesToAvoid = self.pathToGoal + [self.parentDict[nearestBranchVertex]]
        printStatus('Gotta avoid some holes: {0}'.format(nodesToAvoid))
        chosenSidestep = self.findBestSidestepVertexOfBranchVertex(nearestBranchVertex,nodesToAvoid, allowFullBranches=False)

        # Find path from robot to sidestepVertex
        pathToSidestep = self.findPathFromRobotToNode(chosenSidestep)

        # Clear path from robot to sidestepVertex
        self.clearPathFromRobotToNode(chosenSidestep, pathToSidestep)

        # Move robot to sidestepVertex
        self.moveRobotToNode(chosenSidestep)

        return True
           
    def getNearestSidestepVertexBehind(self):
        # Check if the robot is at a branch vertex, if it is, then we consider that
        nearestBranchVertex = None
        if self.robot in self.branchVertexes:
            return self.robot
        else:
            backwardsNode = None
            goalTag = self.nodeTags[self.goal]
            for childNode in self.childrenDict[self.robot]:
                if self.nodeTags[childNode] != goalTag:
                    backwardsNode = childNode
                    break
            if backwardsNode == None:
                return None

            queue = [backwardsNode]
            while len(queue) > 0:
                currentNode = queue.pop(0)
                if len(self.childrenDict[currentNode]) > 1:
                    nearestBranchVertex = currentNode
                    break
                elif len(self.childrenDict[currentNode]) == 1:
                    queue.append(self.childrenDict[currentNode][0])
        return nearestBranchVertex
                    
    def getNearestSidestepVertexAhead(self):
        distance = 0
        furthestBranchVertex = None
        for vertex in self.pathToGoal:
            distance += 1

            # Need to reach branch vertex, then sidestep, so we actually need distance+1 holes
            if distance + 1 > self.frontHoles:
                break
            if vertex in self.branchVertexes:
                furthestBranchVertex = vertex
        return furthestBranchVertex

    def findBestSidestepVertexOfBranchVertex(self, branchVertex, nodesToAvoid, allowFullBranches = True):
        sidestepVertexes = list(map(int,self.graph.neighbors(branchVertex)))
        sidestepVertexes = [x for x in sidestepVertexes if x not in nodesToAvoid and x != self.robot]
        printStatus('Sidestep vertexes of {0}: {1}'.format(branchVertex, sidestepVertexes))

        if not allowFullBranches:
            # Never pick a sidestep in a branch with zero holes
            sidestepVertexesWithBranchSpace = []
            for sidestepVertex in sidestepVertexes:
                holeCount = int(self.stateMap[sidestepVertex] == HOLE_TAG)
                holeCount += len(self.findHolesInChildrenOfNode(sidestepVertex))
                if holeCount > 0:
                    sidestepVertexesWithBranchSpace.append(sidestepVertex)
            sidestepVertexes = sidestepVertexesWithBranchSpace
            printStatus('Ignoring full branches, new sidestep vertexes of {0}: {1}'.format(branchVertex, sidestepVertexes))
            if len(sidestepVertexes) == 0:
                return None
         
        randIndex = random.randint(0,len(sidestepVertexes) - 1)
        chosenSidestep = sidestepVertexes[randIndex]
        
        printStatus('Chosen sidestep vertex: {0}'.format(chosenSidestep))
        return chosenSidestep

    def fillHolesOfSidestepBranchIfNeeded(self, branchNode, sidestepNode):
        holesBehindSidestep = self.findHolesInChildrenOfNode(sidestepNode)
        printStatus('Holes behind {0}: {1}'.format(sidestepNode,holesBehindSidestep))

        availableHoles = self.frontHoles + self.backHoles - len(holesBehindSidestep)
        if self.minHoles > availableHoles:
            # Need to use holes from branch
            printStatus('Need to use holes in branch!')
            obstaclesToRemove = [x for x in self.pathToGoal if self.stateMap[x] == OBSTACLE_TAG]
            if self.stateMap[sidestepNode] == OBSTACLE_TAG:
                obstaclesToRemove.append(sidestepNode)
            distances = [self.distance(sidestepNode,x) for x in obstaclesToRemove]
            obstaclesWithDistances = list(zip(obstaclesToRemove, distances))
            obstaclesWithDistances.sort(key=lambda x: x[1])
            print(obstaclesWithDistances)
            neededHoles = self.minHoles - availableHoles
            count = 0
            while count < neededHoles:
                nearestObstacle = obstaclesWithDistances.pop(0)
                self.moveObstacleToHole(nearestObstacle[0], holesBehindSidestep[count])
                count += 1
    
    def clearPathFromRobotToNode(self, node, pathToNode):
        # Find obstacles in path to node
        obstaclesInPathToNode = [x for x in pathToNode if self.stateMap[x] == OBSTACLE_TAG]

        # Find valid holes related to node (so we don't perform a move that would be blocked by the robot)
        nodelist = list(map(int,self.graph.nodes))
        nodeTag = self.nodeTags[node]
        validHolesOutsidePathToNode = [x for x in nodelist if x not in pathToNode and self.stateMap[x] == HOLE_TAG and self.nodeTags[x] == nodeTag]

        printStatus('Path from robot {0} to node {1}: {2}'.format(self.robot, node, pathToNode))
        printStatus('Obstacles in path: ' + str(obstaclesInPathToNode))
        printStatus('Valid holes: ' + str(validHolesOutsidePathToNode))
        
        # Solve as a min cost flow problem
        m = nx.DiGraph()

        m.add_node('source', demand=-len(obstaclesInPathToNode))
        m.add_node('sink', demand=len(obstaclesInPathToNode))
        for obstacle in obstaclesInPathToNode:
            m.add_edge('source',obstacle,capacity=1, weight=0)
        
        for hole in validHolesOutsidePathToNode:
            m.add_edge(hole,'sink',capacity=1, weight=0)

        for obstacle in obstaclesInPathToNode:
            for hole in validHolesOutsidePathToNode:
                m.add_edge(obstacle,hole,capacity=1, weight=self.distance(obstacle,hole))
        flowDict = nx.min_cost_flow(m)

        for obstacle in obstaclesInPathToNode:
            targetNode = [k for k,v in flowDict[obstacle].items() if v == 1][0]
            self.moveObstacleToHole(obstacle, targetNode)

    def moveObstacleToHole(self, obstacle, hole):
        if self.nodeTags[obstacle] != self.nodeTags[hole]:
            self.raiseInvalidMoveError(obstacle, hole)

        self.obstacles.remove(obstacle)
        self.obstacles.append(hole)
        self.stateMap[obstacle] = HOLE_TAG
        self.stateMap[hole] = OBSTACLE_TAG

        cost = self.distance(obstacle, hole)
        self.totalCost += cost
        self.moves.append((obstacle, hole, cost))
        printStatus('Moving obstacle {0} to hole {1} at cost {2}'.format(obstacle,hole,cost))

        if self.savePicsOfMoves:
            self.currentMove += 1
            self.drawGraph(savingImage=True,isMove=True)

    def moveRobotToNode(self, newRobotNode):
        # Could add validation that this is a valid move
        # Requires there not being any obstacles between self.robot and newRobotNode
        cost = self.distance(self.robot, newRobotNode)
        self.totalCost += cost
        self.moves.append((self.robot, newRobotNode, cost))
        printStatus('Moving robot from {0} to {1} at cost {2}'.format(self.robot,newRobotNode,cost))
        
        self.stateMap[self.robot] = HOLE_TAG
        self.stateMap[newRobotNode] = ROBOT_TAG
        self.robot = newRobotNode
        

        if self.savePicsOfMoves:
            self.currentMove += 1
            self.drawGraph(savingImage=True,isMove=True)
        
    # Finds holes, sorted by closest to the root node
    # Parameter mode:
    # -> 0: consider all children
    # -> 1: consider only children not in the goal branch
    def findHolesInChildrenOfNode(self, node, mode = 0):
        holes = []
        queue = [node]
        goalTag = self.nodeTags[self.goal]
        while len(queue) > 0:
            currentNode = queue.pop(0)
            for child in self.childrenDict[currentNode]:
                if mode == 1 and self.nodeTags[child] == goalTag:
                    continue
                if self.stateMap[child] == HOLE_TAG:
                    holes.append(child)
                queue.append(child)     
        return holes

    def raiseInvalidMoveError(self,nodeA, nodeB):
        raise ValueError('Tried to make invalid move from {0} to {1}'.format(nodeA, nodeB)) 

    def raiseSomethingWrongError(self):
        raise ValueError('Something went wrong. Sorry.') 

def generateInstances(numberOfInstances):
    for i in range(int(numberOfInstances)):
        printStatus("Generating problem " + str(i))
        problem = Problem()
        problem.generateGraph(chainCount= 3+i/10)
        problem.export()
        printStatus('Saving as: ' + problem.name)

def readInstance(fileName, savingPics):
    with open(fileName, 'r') as f:
        problemDict = json.load(f)

    p = Problem()
    p.read(problemDict)
    p.initialize(savePics = savingPics)
    # p.drawGraph()
    startTime = time.time()
    hasSolution = p.tryToSolve()
    endTime = time.time()
    elapsedTime = endTime - startTime
    print('Finished in {0}'.format(elapsedTime))

    if hasSolution:
        print('Solved with cost {0}!'.format(p.totalCost))
        filePathA = 'solutions/' + p.name + '_solution.txt'
        filePathB = 'solutions/solution_' + p.name + '.txt'
        solution = {'moves':p.moves, 'cost':p.totalCost, 'elapsedTime':elapsedTime, 'solvable':True}
    else:
        print('Unsolvable instance :(')
        filePathA = 'solutions/' + p.name + '_unsolvable.txt'
        filePathB = 'solutions/unsolvable_' + p.name + '.txt'
        solution = {'moves':[], 'cost':-1, 'elapsedTime':elapsedTime, 'solvable':False}

    # Save twice just to sort files more conveniently
    with open(filePathA,mode='w') as f:
        f.write(str(solution))
    with open(filePathB,mode='w') as f:
        f.write(str(solution))

# Status printer
def printStatus(text):
    # Uncomment/comment the line below to see or hide debug messages.
    # print(text)
    return

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print('Usage:')
        print('Generate intances: python ' + sys.argv[0] + ' -G [number of instances]')
        print('Read instances: python ' + sys.argv[0] + ' -R [path of instance] [save pics of moves (default: False)]')
        sys.exit(-1)
    
    if not os.path.exists('instances'):
        os.mkdir('instances')
    if not os.path.exists('solutions'):
        os.mkdir('solutions')

    if sys.argv[1] == '-G':
        generateInstances(sys.argv[2])
    
    if sys.argv[1] == '-R':
        shouldSavePics = False
        if len(sys.argv) > 3:
            shouldSavePics = True
        readInstance(sys.argv[2], shouldSavePics)
        