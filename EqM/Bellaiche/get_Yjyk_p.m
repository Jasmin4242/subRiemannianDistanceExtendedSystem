function Yjyk_p =  get_Yjyk_p(vektorfeld_rechts,koordinate_nummer,y,p)

    %zuerst zB Y2y4 bestimmen
    ergebnis_schritt1=vektorfeld_rechts(koordinate_nummer);    
    
    Yjyk_p=subs(ergebnis_schritt1,[y(1) y(2) y(3) y(4)],[p(1) p(2) p(3) p(4)]);

end