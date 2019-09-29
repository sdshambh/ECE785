#include "geometry.h"
#include <arm_neon.h>
const float Lon []= {
		2.66284884	,
		-3.12413936	,
		2.67646241	,
		1.64148216	,
		3.10232275	,
		1.54932878	,
		2.56353961	,
		2.15199097	,
		1.23307512	,
		1.23918377	,
		1.23970737	,
		1.31004414	,
		2.65447126	,
		2.48814138	,
		2.26892803	,
		1.36955986	,
		1.39923046	,
		2.17084052	,
		2.58273823	,
		2.17694918	,
		1.33570048	,
		2.50280215	,
		1.44914688	,
		1.32138878	,
		2.17310945	,
		1.53711147	,
		1.30376095	,
		2.17276039	,
		1.58353723	,
		1.31597826	,
		2.65534392	,
		1.38491876	,
		1.23691484	,
		1.17809725	,
		1.10828408	,
		1.49382731	,
		1.51128060	,
		2.09160258	,
		2.65202780	,
		1.38055544	,
		2.17433118	,
		1.68773339	,
		2.40837983	,
		2.33228348	,
		2.65290046	,
		1.39434354	,
		1.39800873	,
		1.35681896	,
		1.64776535	,
		1.16204022	,
		1.41144777	,
		2.58657795	,
		1.20672064	,
		2.14466058	,
		1.23255152	,
		1.17478112	,
		1.45892072	,
		2.61781935	,
		2.70456221	,
		1.44443449	,
		1.66486957	,
		1.35105937	,
		1.27705741	,
		1.41127323	,
		0.80302599	,
		1.56503674	,
		1.53222455	,
		1.20183372	,
		2.55969988	,
		1.40289565	,
		2.58291276	,
		1.25750973	,
		2.13628300	,
		1.18909282	,
		2.68885425	,
		2.68606172	,
		1.50813901	,
		1.20864051	,
		1.22504660	,
		1.24773588	,
		2.17694918	,
		2.14972204	,
		2.16543000	,
		2.83232031	,
		2.82848059	,
		1.28630766	,
		1.34879045	,
		1.52803576	,
		1.54234746	,
		2.60874363	,
		2.15967042	,
		2.10573974	,
		1.44460902	,
		1.69384204	,
		2.17851997	,
		1.22417394	,
		2.56039801	,
		2.16368467	,
		2.11115026	,
		1.55875355	,
		3.00493337	,
		1.13463855	,
		1.02450827	,
		1.09955743	,
		2.65674019	,
		1.31615279	,
		1.43850037	,
		1.51896005	,
		1.50115769	,
		2.09788576	,
		1.64113310	,
		2.06001212	,
		2.14378792	,
		2.12720729	,
		1.42994826	,
		1.42907559	,
		3.00161725	,
		1.20968770	,
		2.65953271	,
		2.56266694	,
		1.37863558	,
		1.53047922	,
		2.69217037	,
		2.80666397	,
		2.37626578	,
		2.14396245	,
		1.41563656	,
		1.44583075	,
		1.56084795	,
		1.40551365	,
		2.17084052	,
		1.41842908	,
		1.48108640	,
		2.10242362	,
		2.10940493	,
		2.07798901	,
		1.52245071	,
		2.17293492	,
		2.39895506	,
		-3.05921311	,
		2.75430409	,
		2.08619205	,
		2.17712371	,
		1.26710904	,
		1.33412968	,
		2.19614780	,
		1.34303086	,
		1.44583075	,
		1.43902397	,
		1.30620441	,
		2.28585772	,
		0.92519904	,
		1.21562182	,
		1.42244334	,
		1.63484991	,
		2.80404598	,
		1.43815130	,
		1.56713114	,
		2.56284147	,
		2.27695654	,
		2.58797421	,
		1.47515228	,
		2.13698114	,
		1.48457706	,
		0.00000000  
};
const float CosLat []={
	
		0.558179626	,
		0.627283673	,
		0.50934184	,
		0.927053036	,
		0.543613999	,
		0.865238938	,
		0.487250126	,
		0.785424971	,
		0.739043499	,
		0.750226479	,
		0.75011107	,
		0.965518083	,
		0.503623202	,
		0.5613614	,
		0.787688286	,
		0.875380165	,
		0.878650499	,
		0.727293789	,
		0.507538363	,
		0.677518078	,
		0.822938104	,
		0.504678307	,
		0.873432484	,
		0.799579853	,
		0.69289862	,
		0.863835505	,
		0.783042563	,
		0.673270652	,
		0.680976533	,
		0.819051924	,
		0.491663844	,
		0.737395238	,
		0.914465962	,
		0.965880639	,
		0.958819735	,
		0.899023454	,
		0.674560116	,
		0.826589749	,
		0.516234404	,
		0.843391446	,
		0.757564984	,
		0.891244111	,
		0.526213924	,
		0.54068086	,
		0.51009263	,
		0.841605059	,
		0.901907926	,
		0.834463352	,
		0.872666515	,
		0.753448647	,
		0.853550797	,
		0.554844427	,
		0.729088092	,
		0.794838455	,
		0.731710695	,
		0.716058325	,
		0.867591924	,
		0.529327086	,
		0.605432907	,
		0.737630974	,
		0.88376563	,
		0.723931095	,
		0.76323247	,
		0.907484425	,
		0.968756543	,
		0.899557779	,
		0.66835202	,
		0.722001788	,
		0.483587949	,
		0.906234012	,
		0.501057676	,
		0.758248137	,
		0.800835908	,
		0.719703424	,
		0.916711751	,
		0.916642003	,
		0.7028983	,
		0.760405966	,
		0.749649204	,
		0.887252483	,
		0.662750756	,
		0.664839325	,
		0.711903486	,
		0.917407699	,
		0.911259576	,
		0.761877558	,
		0.824323851	,
		0.865413892	,
		0.666272209	,
		0.503924737	,
		0.777585121	,
		0.823334533	,
		0.908581095	,
		0.884336654	,
		0.734322509	,
		0.72501385	,
		0.483893455	,
		0.774944489	,
		0.822144041	,
		0.670815025	,
		0.620737354	,
		0.933642959	,
		0.929454883	,
		0.887010833	,
		0.514289859	,
		0.844514913	,
		0.715936483	,
		0.735269578	,
		0.876390749	,
		0.832244524	,
		0.868804409	,
		0.843485209	,
		0.79058271	,
		0.81167594	,
		0.910683661	,
		0.910322812	,
		0.573433459	,
		0.751724718	,
		0.953559371	,
		0.496519532	,
		0.893371388	,
		0.722363962	,
		0.531102846	,
		0.588773214	,
		0.550043539	,
		0.664969688	,
		0.90901802	,
		0.747450357	,
		0.875380165	,
		0.865676127	,
		0.744894057	,
		0.867244548	,
		0.871128119	,
		0.826393243	,
		0.820451338	,
		0.831566565	,
		0.679697383	,
		0.711535677	,
		0.755967731	,
		0.573576436	,
		0.95584432	,
		0.844047252	,
		0.664056718	,
		0.770735699	,
		0.778243149	,
		0.696038123	,
		0.82698246	,
		0.702774145	,
		0.890451205	,
		0.802713402	,
		0.694532811	,
		0.968234981	,
		0.848233021	,
		0.95716803	,
		0.90039472	,
		0.945006008	,
		0.746870346	,
		0.677774777	,
		0.490903754	,
		0.736451398	,
		0.488012005	,
		0.878817113	,
		0.673528709	,
		0.94046622	,
		1		

	
};
const float SinLat []={
		0.829720137	,
		0.778790853	,
		0.860564286	,
		0.374930219	,
		0.839335344	,
		0.501359732	,
		0.873262455	,
		0.618956876	,
		0.673657707	,
		0.661180936	,
		0.661311865	,
		0.260335998	,
		0.863923417	,
		0.82757077	,
		0.616073992	,
		0.483435173	,
		0.477465496	,
		0.686326267	,
		0.86162916	,
		0.735506121	,
		0.568131039	,
		0.863307481	,
		0.48694527	,
		0.600559787	,
		0.721035022	,
		0.503773977	,
		0.621968121	,
		0.739396124	,
		0.732305238	,
		0.573719397	,
		0.870785085	,
		0.675461518	,
		0.404662829	,
		0.258987627	,
		0.284015345	,
		0.437900479	,
		0.73821992	,
		0.562804928	,
		0.856447337	,
		0.537299608	,
		0.652759752	,
		0.453523908	,
		0.850352225	,
		0.841227797	,
		0.860119473	,
		0.540093441	,
		0.431928343	,
		0.55106344	,
		0.488316653	,
		0.657506757	,
		0.521009632	,
		0.831954122	,
		0.684419866	,
		0.606821086	,
		0.681615331	,
		0.698040454	,
		0.497276838	,
		0.848417843	,
		0.795896347	,
		0.675204078	,
		0.467929814	,
		0.689872285	,
		0.64612398	,
		0.420085728	,
		0.248013628	,
		0.436801788	,
		0.74384513	,
		0.69189119	,
		0.875295776	,
		0.422776436	,
		0.865413892	,
		0.651966074	,
		0.598883835	,
		0.69428163	,
		0.399549203	,
		0.399709193	,
		0.711290363	,
		0.649448048	,
		0.66183538	,
		0.461284112	,
		0.74884006	,
		0.746986394	,
		0.702277314	,
		0.397948631	,
		0.411832473	,
		0.647721072	,
		0.566118528	,
		0.501057676	,
		0.745708618	,
		0.863747567	,
		0.628777688	,
		0.567556382	,
		0.417708503	,
		0.466849742	,
		0.678800746	,
		0.688734286	,
		0.875126919	,
		0.632029303	,
		0.569279523	,
		0.741624705	,
		0.784018582	,
		0.358205004	,
		0.368935795	,
		0.461748613	,
		0.85761643	,
		0.535532036	,
		0.698165419	,
		0.677774777	,
		0.481600722	,
		0.554408741	,
		0.495155429	,
		0.537152401	,
		0.612355272	,
		0.58410801	,
		0.41310443	,
		0.413898994	,
		0.81925214	,
		0.659477026	,
		0.301205123	,
		0.868025549	,
		0.449318999	,
		0.691513056	,
		0.847307363	,
		0.808298276	,
		0.83513598	,
		0.746870346	,
		0.41675681	,
		0.664317668	,
		0.483435173	,
		0.500604478	,
		0.667182767	,
		0.49788241	,
		0.491055802	,
		0.563093428	,
		0.571716365	,
		0.555425106	,
		0.73349265	,
		0.70264997	,
		0.654608882	,
		0.819152044	,
		0.293873503	,
		0.536268811	,
		0.747682202	,
		0.637154991	,
		0.627963058	,
		0.718004827	,
		0.562227722	,
		0.711413031	,
		0.45507873	,
		0.596364984	,
		0.71946103	,
		0.250042042	,
		0.529623207	,
		0.289533009	,
		0.435073957	,
		0.327052969	,
		0.664969688	,
		0.735269578	,
		0.871213811	,
		0.676490457	,
		0.872836916	,
		0.47715876	,
		0.739161063	,
		0.33988717	,
		0		
	
};
								
const float Lat [] ={
		0.97860611	,
		0.89273591	,
		1.03637651	,
		0.38432150	,
		0.99605940	,
		0.52516957	,
		1.06185832	,
		0.66741391	,
		0.73914694	,
		0.72239178	,
		0.72256631	,
		0.26337018	,
		1.04300876	,
		0.97476639	,
		0.66374871	,
		0.50457469	,
		0.49776790	,
		0.75642570	,
		1.03847090	,
		0.82641340	,
		0.60423299	,
		1.04178703	,
		0.50858894	,
		0.64420103	,
		0.80529492	,
		0.52796210	,
		0.67125363	,
		0.83217299	,
		0.82170101	,
		0.61103977	,
		1.05679686	,
		0.74159040	,
		0.41661009	,
		0.26197392	,
		0.28797933	,
		0.45326201	,
		0.83042766	,
		0.59777527	,
		1.02834800	,
		0.56723201	,
		0.71122167	,
		0.47071530	,
		1.01665429	,
		0.99955006	,
		1.03550385	,
		0.57054813	,
		0.44662976	,
		0.58363810	,
		0.51015974	,
		0.71750486	,
		0.54803339	,
		0.98262037	,
		0.75380770	,
		0.65205501	,
		0.74996798	,
		0.77265726	,
		0.52045718	,
		1.01298910	,
		0.92048665	,
		0.74124133	,
		0.48694686	,
		0.76131262	,
		0.70249502	,
		0.43353979	,
		0.25062928	,
		0.45204028	,
		0.83880524	,
		0.76410515	,
		1.06604711	,
		0.43650685	,
		1.04597582	,
		0.71017447	,
		0.64210663	,
		0.76742127	,
		0.41102504	,
		0.41119957	,
		0.79133228	,
		0.70685835	,
		0.72326444	,
		0.47944195	,
		0.84631015	,
		0.84351763	,
		0.77859138	,
		0.40927971	,
		0.42446407	,
		0.70458942	,
		0.60178953	,
		0.52482051	,
		0.84159777	,
		1.04265970	,
		0.67998028	,
		0.60353486	,
		0.43092179	,
		0.48572513	,
		0.74612826	,
		0.75974182	,
		1.06569804	,
		0.68416907	,
		0.60562925	,
		0.83548911	,
		0.90111349	,
		0.36634461	,
		0.37786378	,
		0.47996554	,
		1.03061692	,
		0.56513761	,
		0.77283179	,
		0.74473199	,
		0.50248029	,
		0.58765236	,
		0.51801372	,
		0.56705747	,
		0.65903633	,
		0.62378067	,
		0.42586034	,
		0.42673300	,
		0.96010562	,
		0.72012285	,
		0.30595622	,
		1.05121181	,
		0.46600291	,
		0.76358155	,
		1.01089470	,
		0.94125607	,
		0.98837996	,
		0.84334309	,
		0.42987459	,
		0.72658057	,
		0.50457469	,
		0.52429691	,
		0.73042029	,
		0.52115531	,
		0.51330133	,
		0.59812433	,
		0.60859631	,
		0.58887409	,
		0.82344634	,
		0.77911498	,
		0.71366513	,
		0.95993109	,
		0.29827677	,
		0.56601028	,
		0.84456483	,
		0.69080132	,
		0.67893308	,
		0.80093159	,
		0.59707714	,
		0.79150682	,
		0.47246063	,
		0.63896504	,
		0.80302599	,
		0.25272368	,
		0.55815629	,
		0.29373891	,
		0.45012041	,
		0.33318335	,
		0.72745323	,
		0.82606433	,
		1.05766953	,
		0.74298666	,
		1.06098565	,
		0.49741884	,
		0.83182392	,
		0.34679692	,
		0.00000000	
	};
const PT_T waypoints[] = {	//	Lat 		sin(Lat)		cos(Lat)		Lon		Name	
	{	0.97860611	,	0.829720137	,	0.558179626	,	2.66284884	,	"ALBATROSS BNK"	},
	{	0.89273591	,	0.778790853	,	0.627283673	,	-3.12413936	,	"AMCHITKA"	},
	{	1.03637651	,	0.860564286	,	0.50934184	,	2.67646241	,	"AUGUSTINE ISLAND, AK"	},
	{	0.38432150	,	0.374930219	,	0.927053036	,	1.64148216	,	"BAY CAMPECHE"	},
	{	0.99605940	,	0.839335344	,	0.543613999	,	3.10232275	,	"BERING SEA"	},
	{	0.52516957	,	0.501359732	,	0.865238938	,	1.54932878	,	"BILOXI"	},
	{	1.06185832	,	0.873262455	,	0.487250126	,	2.56353961	,	"BLIGH REEF LIGHT, AK"	},
	{	0.66741391	,	0.618956876	,	0.785424971	,	2.15199097	,	"BODEGA BAY"	},
	{	0.73914694	,	0.673657707	,	0.739043499	,	1.23307512	,	"BOSTON"	},
	{	0.72239178	,	0.661180936	,	0.750226479	,	1.23918377	,	"BUZZARDS BAY"	},
	{	0.72256631	,	0.661311865	,	0.75011107	,	1.23970737	,	"BUZZARDS BAY,MA"	},
	{	0.26337018	,	0.260335998	,	0.965518083	,	1.31004414	,	"C CARIBBEAN"	},
	{	1.04300876	,	0.863923417	,	0.503623202	,	2.65447126	,	"C COOK INL"	},
	{	0.97476639	,	0.82757077	,	0.5613614	,	2.48814138	,	"C GULF ALASKA"	},
	{	0.66374871	,	0.616073992	,	0.787688286	,	2.26892803	,	"CALIFORNIA"	},
	{	0.50457469	,	0.483435173	,	0.875380165	,	1.36955986	,	"CANAVERAL E"	},
	{	0.49776790	,	0.477465496	,	0.878650499	,	1.39923046	,	"CANAVERAL W"	},
	{	0.75642570	,	0.686326267	,	0.727293789	,	2.17084052	,	"CAPE ARAGO ,OR"	},
	{	1.03847090	,	0.86162916	,	0.507538363	,	2.58273823	,	"CAPE CLEARE"	},
	{	0.82641340	,	0.735506121	,	0.677518078	,	2.17694918	,	"CAPE ELIZABETH"	},
	{	0.60423299	,	0.568131039	,	0.822938104	,	1.33570048	,	"CAPE LOOKOUT, NC"	},
	{	1.04178703	,	0.863307481	,	0.504678307	,	2.50280215	,	"CAPE SUCKLING"	},
	{	0.50858894	,	0.48694527	,	0.873432484	,	1.44914688	,	"CEDAR KEY, FL"	},
	{	0.64420103	,	0.600559787	,	0.799579853	,	1.32138878	,	"CHESAPEAKE LIGHT, VA"	},
	{	0.80529492	,	0.721035022	,	0.69289862	,	2.17310945	,	"COL RIVER BAR"	},
	{	0.52796210	,	0.503773977	,	0.863835505	,	1.53711147	,	"DAUPHIN ISLAND, AL"	},
	{	0.67125363	,	0.621968121	,	0.783042563	,	1.30376095	,	"DELAWARE BAY"	},
	{	0.83217299	,	0.739396124	,	0.673270652	,	2.17276039	,	"DESTRUCTION ISLAND,WA"	},
	{	0.82170101	,	0.732305238	,	0.680976533	,	1.58353723	,	"DEVILS ISLAND, WI"	},
	{	0.61103977	,	0.573719397	,	0.819051924	,	1.31597826	,	"DIAMOND SHLS"	},
	{	1.05679686	,	0.870785085	,	0.491663844	,	2.65534392	,	"DRIFT RIVER TERMINAL,"	},
	{	0.74159040	,	0.675461518	,	0.737395238	,	1.38491876	,	"DUNKIRK, NY"	},
	{	0.41661009	,	0.404662829	,	0.914465962	,	1.23691484	,	"E BAHAMAS"	},
	{	0.26197392	,	0.258987627	,	0.965880639	,	1.17809725	,	"E CARIBBEAN"	},
	{	0.28797933	,	0.284015345	,	0.958819735	,	1.10828408	,	"E CARIBBEAN"	},
	{	0.45326201	,	0.437900479	,	0.899023454	,	1.49382731	,	"E GULF MEXICO"	},
	{	0.83042766	,	0.73821992	,	0.674560116	,	1.51128060	,	"E LK SUPERIOR"	},
	{	0.59777527	,	0.562804928	,	0.826589749	,	2.09160258	,	"E STA BARBARA"	},
	{	1.02834800	,	0.856447337	,	0.516234404	,	2.65202780	,	"EAST AMATULI ISLAND L"	},
	{	0.56723201	,	0.537299608	,	0.843391446	,	1.38055544	,	"EDISTO"	},
	{	0.71122167	,	0.652759752	,	0.757564984	,	2.17433118	,	"EEL RIVER"	},
	{	0.47071530	,	0.453523908	,	0.891244111	,	1.68773339	,	"EILEEN"	},
	{	1.01665429	,	0.850352225	,	0.526213924	,	2.40837983	,	"FAIRWEATHER"	},
	{	0.99955006	,	0.841227797	,	0.54068086	,	2.33228348	,	"FIVE FINGERS, AK"	},
	{	1.03550385	,	0.860119473	,	0.51009263	,	2.65290046	,	"FLAT ISLAND LIGHT"	},
	{	0.57054813	,	0.540093441	,	0.841605059	,	1.39434354	,	"FOLLY ISLAND, SC"	},
	{	0.44662976	,	0.431928343	,	0.901907926	,	1.39800873	,	"FOWEY ROCK, FL"	},
	{	0.58363810	,	0.55106344	,	0.834463352	,	1.35681896	,	"FRYING PAN SHLS"	},
	{	0.51015974	,	0.488316653	,	0.872666515	,	1.64776535	,	"GALVESTON"	},
	{	0.71750486	,	0.657506757	,	0.753448647	,	1.16204022	,	"GEORGES BNK"	},
	{	0.54803339	,	0.521009632	,	0.853550797	,	1.41144777	,	"GRAYS REEF"	},
	{	0.98262037	,	0.831954122	,	0.554844427	,	2.58657795	,	"GULF ALASKA"	},
	{	0.75380770	,	0.684419866	,	0.729088092	,	1.20672064	,	"GULF MAINE"	},
	{	0.65205501	,	0.606821086	,	0.794838455	,	2.14466058	,	"HALF MOON BAY"	},
	{	0.74996798	,	0.681615331	,	0.731710695	,	1.23255152	,	"ISLE OF SHOALS, NH"	},
	{	0.77265726	,	0.698040454	,	0.716058325	,	1.17478112	,	"JONESPORT"	},
	{	0.52045718	,	0.497276838	,	0.867591924	,	1.45892072	,	"KEATON BEACH, FL"	},
	{	1.01298910	,	0.848417843	,	0.529327086	,	2.61781935	,	"KENNEDY ENTR"	},
	{	0.92048665	,	0.795896347	,	0.605432907	,	2.70456221	,	"KODIAK"	},
	{	0.74124133	,	0.675204078	,	0.737630974	,	1.44443449	,	"LAKE ST. CLAIR"	},
	{	0.48694686	,	0.467929814	,	0.88376563	,	1.66486957	,	"LANEILLE"	},
	{	0.76131262	,	0.689872285	,	0.723931095	,	1.35105937	,	"LK ONTARIO"	},
	{	0.70249502	,	0.64612398	,	0.76323247	,	1.27705741	,	"LONG ISLAND"	},
	{	0.43353979	,	0.420085728	,	0.907484425	,	1.41127323	,	"LONG KEY, FL"	},
	{	0.25062928	,	0.248013628	,	0.968756543	,	0.80302599	,	"M ATLANTIC"	},
	{	0.45204028	,	0.436801788	,	0.899557779	,	1.56503674	,	"M GULF MEXICO"	},
	{	0.83880524	,	0.74384513	,	0.66835202	,	1.53222455	,	"M LK SUPERIOR"	},
	{	0.76410515	,	0.69189119	,	0.722001788	,	1.20183372	,	"MATINICUS ROCK, ME"	},
	{	1.06604711	,	0.875295776	,	0.483587949	,	2.55969988	,	"MIDDLE ROCK LIGHT, AK"	},
	{	0.43650685	,	0.422776436	,	0.906234012	,	1.40289565	,	"MOLASSES REEF, FL"	},
	{	1.04597582	,	0.865413892	,	0.501057676	,	2.58291276	,	"MONTAGUE STR"	},
	{	0.71017447	,	0.651966074	,	0.758248137	,	1.25750973	,	"MONTAUK PT"	},
	{	0.64210663	,	0.598883835	,	0.800835908	,	2.13628300	,	"MONTEREY BAY"	},
	{	0.76742127	,	0.69428163	,	0.719703424	,	1.18909282	,	"MT DESERT ROCK, ME"	},
	{	0.41102504	,	0.399549203	,	0.916711751	,	2.68885425	,	"N HAWAII"	},
	{	0.41119957	,	0.399709193	,	0.916642003	,	2.68606172	,	"N HAWAII"	},
	{	0.79133228	,	0.711290363	,	0.7028983	,	1.50813901	,	"N LK MICHIGAN"	},
	{	0.70685835	,	0.649448048	,	0.760405966	,	1.20864051	,	"NANTUCKET"	},
	{	0.72326444	,	0.66183538	,	0.749649204	,	1.22504660	,	"NANTUCKET SND"	},
	{	0.47944195	,	0.461284112	,	0.887252483	,	1.24773588	,	"NE BAHAMAS"	},
	{	0.84631015	,	0.74884006	,	0.662750756	,	2.17694918	,	"NEAH BAY"	},
	{	0.84351763	,	0.746986394	,	0.664839325	,	2.14972204	,	"NEW DUNGENESS"	},
	{	0.77859138	,	0.702277314	,	0.711903486	,	2.16543000	,	"NEWPORT, OR"	},
	{	0.40927971	,	0.397948631	,	0.917407699	,	2.83232031	,	"NW HAWAII"	},
	{	0.42446407	,	0.411832473	,	0.911259576	,	2.82848059	,	"NW HAWAII"	},
	{	0.70458942	,	0.647721072	,	0.761877558	,	1.28630766	,	"NYC ENTR"	},
	{	0.60178953	,	0.566118528	,	0.824323851	,	1.34879045	,	"ONSLOW BAY"	},
	{	0.52482051	,	0.501057676	,	0.865413892	,	1.52803576	,	"ORANGE BCH"	},
	{	0.84159777	,	0.745708618	,	0.666272209	,	1.54234746	,	"PASSAGE ISLAND, MI"	},
	{	1.04265970	,	0.863747567	,	0.503924737	,	2.60874363	,	"PILOT ROCK, AK"	},
	{	0.67998028	,	0.628777688	,	0.777585121	,	2.15967042	,	"POINT ARENA, CA"	},
	{	0.60353486	,	0.567556382	,	0.823334533	,	2.10573974	,	"POINT ARGUELLO, CA"	},
	{	0.43092179	,	0.417708503	,	0.908581095	,	1.44460902	,	"POLASKI SHOALS, FL"	},
	{	0.48572513	,	0.466849742	,	0.884336654	,	1.69384204	,	"PORT ARANSAS, TX"	},
	{	0.74612826	,	0.678800746	,	0.734322509	,	2.17851997	,	"PORT ORFORD"	},
	{	0.75974182	,	0.688734286	,	0.72501385	,	1.22417394	,	"PORTLAND"	},
	{	1.06569804	,	0.875126919	,	0.483893455	,	2.56039801	,	"POTATO POINT, AK"	},
	{	0.68416907	,	0.632029303	,	0.774944489	,	2.16368467	,	"PT ARENA"	},
	{	0.60562925	,	0.569279523	,	0.822144041	,	2.11115026	,	"PT ARGUELLO"	},
	{	0.83548911	,	0.741624705	,	0.670815025	,	1.55875355	,	"ROCK OF AGES, MI"	},
	{	0.90111349	,	0.784018582	,	0.620737354	,	3.00493337	,	"S ALEUTIANS"	},
	{	0.36634461	,	0.358205004	,	0.933642959	,	1.13463855	,	"S ATLANTIC"	},
	{	0.37786378	,	0.368935795	,	0.929454883	,	1.02450827	,	"S ATLANTIC"	},
	{	0.47996554	,	0.461748613	,	0.887010833	,	1.09955743	,	"S ATLANTIC"	},
	{	1.03061692	,	0.85761643	,	0.514289859	,	2.65674019	,	"S COOK INL"	},
	{	0.56513761	,	0.535532036	,	0.844514913	,	1.31615279	,	"S HATTERAS"	},
	{	0.77283179	,	0.698165419	,	0.715936483	,	1.43850037	,	"S LK HURON"	},
	{	0.74473199	,	0.677774777	,	0.735269578	,	1.51896005	,	"S LK MICHIGAN"	},
	{	0.50248029	,	0.481600722	,	0.876390749	,	1.50115769	,	"S PENSACOLA"	},
	{	0.58765236	,	0.554408741	,	0.832244524	,	2.09788576	,	"S STA ROSA"	},
	{	0.51801372	,	0.495155429	,	0.868804409	,	1.64113310	,	"SABINE, TX"	},
	{	0.56705747	,	0.537152401	,	0.843485209	,	2.06001212	,	"SAN CLEMENTE"	},
	{	0.65903633	,	0.612355272	,	0.79058271	,	2.14378792	,	"SAN FRANCISCO"	},
	{	0.62378067	,	0.58410801	,	0.81167594	,	2.12720729	,	"SAN MARTIN"	},
	{	0.42586034	,	0.41310443	,	0.910683661	,	1.42994826	,	"SAND KEY"	},
	{	0.42673300	,	0.413898994	,	0.910322812	,	1.42907559	,	"SAND KEY, FL"	},
	{	0.96010562	,	0.81925214	,	0.573433459	,	3.00161725	,	"SE BERING SEA"	},
	{	0.72012285	,	0.659477026	,	0.751724718	,	1.20968770	,	"SE CAPE COD"	},
	{	0.30595622	,	0.301205123	,	0.953559371	,	2.65953271	,	"SE HAWAII"	},
	{	1.05121181	,	0.868025549	,	0.496519532	,	2.56266694	,	"SEAL ROCKS"	},
	{	0.46600291	,	0.449318999	,	0.893371388	,	1.37863558	,	"SETTLEMENT PT, GBI"	},
	{	0.76358155	,	0.691513056	,	0.722363962	,	1.53047922	,	"SHEBOYGAN, WI"	},
	{	1.01089470	,	0.847307363	,	0.531102846	,	2.69217037	,	"SHELIKOF STR"	},
	{	0.94125607	,	0.808298276	,	0.588773214	,	2.80666397	,	"SHUMAGIN IS"	},
	{	0.98837996	,	0.83513598	,	0.550043539	,	2.37626578	,	"SITKA SND"	},
	{	0.84334309	,	0.746870346	,	0.664969688	,	2.14396245	,	"SMITH ISLAND, WA"	},
	{	0.42987459	,	0.41675681	,	0.90901802	,	1.41563656	,	"SOMBRERO KEY, FL"	},
	{	0.72658057	,	0.664317668	,	0.747450357	,	1.44583075	,	"SOUTH BASS ISLAND, OH"	},
	{	0.50457469	,	0.483435173	,	0.875380165	,	1.56084795	,	"SOUTHWEST PASS , LA"	},
	{	0.52429691	,	0.500604478	,	0.865676127	,	1.40551365	,	"ST AUGUSTINE"	},
	{	0.73042029	,	0.667182767	,	0.744894057	,	2.17084052	,	"ST GEORGES"	},
	{	0.52115531	,	0.49788241	,	0.867244548	,	1.41842908	,	"ST. AUGUSTINE, FL"	},
	{	0.51330133	,	0.491055802	,	0.871128119	,	1.48108640	,	"ST. GEORGE OFFSHORE"	},
	{	0.59812433	,	0.563093428	,	0.826393243	,	2.10242362	,	"STA BARBARA"	},
	{	0.60859631	,	0.571716365	,	0.820451338	,	2.10940493	,	"STA MARIA"	},
	{	0.58887409	,	0.555425106	,	0.831566565	,	2.07798901	,	"STA MONICA"	},
	{	0.82344634	,	0.73349265	,	0.679697383	,	1.52245071	,	"STANNARD ROCK, MI"	},
	{	0.77911498	,	0.70264997	,	0.711535677	,	2.17293492	,	"STONEWALL BNK"	},
	{	0.71366513	,	0.654608882	,	0.755967731	,	2.39895506	,	"SW ASTORIA"	},
	{	0.95993109	,	0.819152044	,	0.573576436	,	-3.05921311	,	"SW BERING SEA"	},
	{	0.29827677	,	0.293873503	,	0.95584432	,	2.75430409	,	"SW HAWAII"	},
	{	0.56601028	,	0.536268811	,	0.844047252	,	2.08619205	,	"TANNER BANK"	},
	{	0.84456483	,	0.747682202	,	0.664056718	,	2.17712371	,	"TATOOSH ISLAND, WA"	},
	{	0.69080132	,	0.637154991	,	0.770735699	,	1.26710904	,	"TEXAS TWR 4"	},
	{	0.67893308	,	0.627963058	,	0.778243149	,	1.33412968	,	"THOMAS POINT, MD"	},
	{	0.80093159	,	0.718004827	,	0.696038123	,	2.19614780	,	"TILLAMOOK"	},
	{	0.59707714	,	0.562227722	,	0.82698246	,	1.34303086	,	"UNWC"	},
	{	0.79150682	,	0.711413031	,	0.702774145	,	1.44583075	,	"V N LK HURON"	},
	{	0.47246063	,	0.45507873	,	0.890451205	,	1.43902397	,	"VENICE, FL"	},
	{	0.63896504	,	0.596364984	,	0.802713402	,	1.30620441	,	"VIRGINIA BCH"	},
	{	0.80302599	,	0.71946103	,	0.694532811	,	2.28585772	,	"W ASTORIA"	},
	{	0.25272368	,	0.250042042	,	0.968234981	,	0.92519904	,	"W ATLANTIC"	},
	{	0.55815629	,	0.529623207	,	0.848233021	,	1.21562182	,	"W BERMUDA"	},
	{	0.29373891	,	0.289533009	,	0.95716803	,	1.42244334	,	"W CARIBBEAN"	},
	{	0.45012041	,	0.435073957	,	0.90039472	,	1.63484991	,	"W GULF MEXICO"	},
	{	0.33318335	,	0.327052969	,	0.945006008	,	2.80404598	,	"W HAWAII"	},
	{	0.72745323	,	0.664969688	,	0.746870346	,	1.43815130	,	"W LK ERIE"	},
	{	0.82606433	,	0.735269578	,	0.677774777	,	1.56713114	,	"W LK SUPERIOR"	},
	{	1.05766953	,	0.871213811	,	0.490903754	,	2.56284147	,	"W ORCA BAY"	},
	{	0.74298666	,	0.676490457	,	0.736451398	,	2.27695654	,	"W OREGON"	},
	{	1.06098565	,	0.872836916	,	0.488012005	,	2.58797421	,	"W PWS"	},
	{	0.49741884	,	0.47715876	,	0.878817113	,	1.47515228	,	"W TAMPA"	},
	{	0.83182392	,	0.739161063	,	0.673528709	,	2.13698114	,	"WEST POINT, WA"	},
	{	0.34679692	,	0.33988717	,	0.94046622	,	1.48457706	,	"YUCATAN CHNL"	},
	{	0.00000000	,	0	,	1	,	0.00000000	,	"END"	},

};

const CT_T capitals[] = {	//	Lat		Lon		Name	
	{	42.65982900	,	73.78133900	,	"Albany , New York"},
	{	38.97294500	,	76.50115700	,	"Annapolis , Maryland"},
	{	33.76000000	,	84.39000000	,	"Atlanta , Georgia"},
	{	44.32353500	,	69.76526100	,	"Augusta , Maine"},
	{	30.26666700	,	97.75000000	,	"Austin , Texas"},
	{	30.45809000	,	91.14022900	,	"Baton Rouge, Louisiana"},
	{	48.81334300	,	100.77900400	,	"Bismarck , North Dakota"},
	{	43.61373900	,	116.23765100	,	"Boise , Idaho"},
	{	42.23520000	,	71.02750000	,	"Boston , Massachusetts"},
	{	39.16094900	,	119.75387700	,	"Carson City, Nevada"},
	{	38.34949700	,	81.63329400	,	"Charleston , West Virginia"},
	{	41.14554800	,	104.80204200	,	"Cheyenne , Wyoming"},
	{	34.00000000	,	81.03500000	,	"Columbia , South Carolina"},
	{	39.96224500	,	83.00064700	,	"Columbus , Ohio"},
	{	43.22009300	,	71.54912700	,	"Concord , New Hampshire"},
	{	39.73916670	,	104.98416700	,	"Denver , Colorado"},
	{	41.59093900	,	93.62086600	,	"Des Moines, Iowa"},
	{	39.16192100	,	75.52675500	,	"Dover , Delaware"},
	{	38.19727400	,	84.86311000	,	"Frankfort , Kentucky"},
	{	40.26978900	,	76.87561300	,	"Harrisburg , Pennsylvania"},
	{	41.76700000	,	72.67700000	,	"Hartford , Connecticut"},
	{	46.59580500	,	112.02703100	,	"Helena , Montana"},
	{	21.30895000	,	157.82618200	,	"Honolulu , Hawaii"},
	{	39.79094200	,	86.14768500	,	"Indianapolis , Indiana"},
	{	32.32000000	,	90.20700000	,	"Jackson , Mississippi"},
	{	38.57295400	,	92.18928300	,	"Jefferson City, Missouri"},
	{	58.30193500	,	134.41974000	,	"Juneau , Alaska"},
	{	42.73350000	,	84.54670000	,	"Lansing , Michigan"},
	{	40.80986800	,	96.67534500	,	"Lincoln , Nebraska"},
	{	34.73600900	,	92.33112200	,	"Little Rock, Arkansas"},
	{	43.07472200	,	89.38444400	,	"Madison , Wisconsin"},
	{	32.36153800	,	86.27911800	,	"Montgomery , Alabama"},
	{	44.26639000	,	72.57194000	,	"Montpelier , Vermont"},
	{	36.16500000	,	86.78400000	,	"Nashville , Tennessee"},
	{	35.48230900	,	97.53499400	,	"Oklahoma City, Oklahoma"},
	{	47.04241800	,	122.89307700	,	"Olympia , Washington"},
	{	33.44845700	,	112.07384400	,	"Phoenix , Arizona"},
	{	44.36796600	,	100.33637800	,	"Pierre , South Dakota"},
	{	41.82355000	,	71.42213200	,	"Providence , Rhode Island"},
	{	35.77100000	,	78.63800000	,	"Raleigh , North Carolina"},
	{	37.54000000	,	77.46000000	,	"Richmond , Virginia"},
	{	38.55560500	,	121.46892600	,	"Sacramento , California"},
	{	44.95000000	,	93.09400000	,	"Saint Paul, Minnesota"},
	{	44.93110900	,	123.02915900	,	"Salem , Oregon"},
	{	40.75470000	,	111.89262200	,	"Salt Lake, Utah"},
	{	35.66723100	,	105.96457500	,	"Santa Fe, New Mexico"},
	{	39.78325000	,	89.65037300	,	"Springfield , Illinois"},
	{	30.45180000	,	84.27277000	,	"Tallahassee , Florida"},
	{	39.04000000	,	95.69000000	,	"Topeka , Kansas"},
	{	40.22174100	,	74.75613800	,	"Trenton , New Jersey"},
	{	0.00000000	,	0	,	"END"	},
};